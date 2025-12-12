import unittest
from unittest.mock import patch, MagicMock
from backend.main import get_all_urls, extract_text_from_url, chunk_text, embed, create_collection, save_chunk_to_qdrant, COHERE_API_KEY, QDRANT_API_KEY, QDRANT_URL
import os
import cohere
from qdrant_client import QdrantClient, models
from qdrant_client.http.models import PointStruct
import uuid
import pytest

# Mock Cohere client for integration tests
class MockCohereClient:
    def __init__(self, api_key):
        self.api_key = api_key
    
    def embed(self, texts, model, input_type):
        if not self.api_key:
            raise cohere.CohereError("API Key is missing")
        if not texts:
            return MagicMock(embeddings=[])
        # Simulate Cohere response: return dummy embeddings of size 1024
        return MagicMock(embeddings=[[0.1] * 1024 for _ in texts])

# Mock Qdrant Client for unit tests
class MockQdrantClient:
    def __init__(self, url, api_key):
        self.url = url
        self.api_key = api_key
        self.collections = {}

    def recreate_collection(self, collection_name, vectors_config):
        if not self.api_key:
            raise Exception("Qdrant API Key is missing")
        self.collections[collection_name] = {
            "vectors_config": vectors_config,
            "points": []
        }
        return True
    
    def upsert(self, collection_name, points, wait):
        if collection_name not in self.collections:
            raise Exception(f"Collection {collection_name} does not exist")
        self.collections[collection_name]["points"].extend(points)
        return True


class TestRAGIngestionPipeline(unittest.TestCase):

    @patch('requests.get')
    def test_get_all_urls_single_page(self, mock_requests_get):
        # Mock a single page with no internal links
        mock_response = MagicMock()
        mock_response.raise_for_status.return_value = None
        mock_response.text = """
        <html>
            <body>
                <a href="/about">About</a>
                <a href="http://external.com">External</a>
            </body>
        </html>
        """
        mock_requests_get.return_value = mock_response

        base_url = "http://test.com/"
        urls = get_all_urls(base_url)
        self.assertIn(base_url, urls)
        self.assertNotIn("http://test.com/about", urls) # Not discovered yet in this simple mock
        self.assertNotIn("http://external.com", urls)
        mock_requests_get.assert_called_once_with(base_url, timeout=5)

    @patch('requests.get')
    def test_get_all_urls_recursive_discovery(self, mock_requests_get):
        # Mock multiple pages to simulate crawling
        def mock_get_side_effect(url, timeout):
            mock_response = MagicMock()
            mock_response.raise_for_status.return_value = None
            if url == "http://test.com/":
                mock_response.text = """
                <html><body><a href="/page1">Page 1</a><a href="/page2">Page 2</a></body></html>
                """
            elif url == "http://test.com/page1":
                mock_response.text = """
                <html><body><a href="/page3">Page 3</a></body></html>
                """
            elif url == "http://test.com/page2":
                mock_response.text = """
                <html><body><a href="http://other.com/external">External</a></body></html>
                """
            elif url == "http://test.com/page3":
                mock_response.text = """
                <html><body>No links here.</body></html>
                """
            else:
                raise requests.exceptions.RequestException("Unexpected URL")
            return mock_response
        
        mock_requests_get.side_effect = mock_get_side_effect

        base_url = "http://test.com/"
        urls = get_all_urls(base_url)
        
        expected_urls = {
            "http://test.com/", 
            "http://test.com/page1", 
            "http://test.com/page2", 
            "http://test.com/page3"
        }
        self.assertSetEqual(set(urls), expected_urls)
        self.assertEqual(mock_requests_get.call_count, 4) # Called for base and 3 discovered pages

    @patch('requests.get')
    def test_extract_text_from_url_basic(self, mock_requests_get):
        mock_response = MagicMock()
        mock_response.raise_for_status.return_value = None
        mock_response.text = """
        <html>
            <head><title>Test Page</title></head>
            <body>
                <main>
                    <h1>Welcome</h1>
                    <p>This is some content.</p>
                    <script>alert('hello');</script>
                    <style>body{color:red;}</style>
                </main>
                <footer>Footer text</footer>
            </body>
        </html>
        """
        mock_requests_get.return_value = mock_response

        url = "http://test.com/test_page"
        extracted_text = extract_text_from_url(url)
        self.assertIn("Welcome This is some content.", extracted_text)
        self.assertNotIn("alert('hello');", extracted_text)
        self.assertNotIn("body{color:red;}", extracted_text)
        self.assertNotIn("Footer text", extracted_text) # Should be excluded by main tag
        mock_requests_get.assert_called_once_with(url, timeout=5)

    @patch('requests.get')
    def test_extract_text_from_url_no_content(self, mock_requests_get):
        mock_requests_get.side_effect = requests.exceptions.RequestException("Connection error")

        url = "http://test.com/error_page"
        extracted_text = extract_text_from_url(url)
        self.assertEqual(extracted_text, "")
        mock_requests_get.assert_called_once_with(url, timeout=5)

    @patch('requests.get')
    def test_extract_text_from_url_empty_main(self, mock_requests_get):
        mock_response = MagicMock()
        mock_response.raise_for_status.return_value = None
        mock_response.text = """
        <html><body><main></main></body></html>
        """
        mock_requests_get.return_value = mock_response

        url = "http://test.com/empty_main"
        extracted_text = extract_text_from_url(url)
        self.assertEqual(extracted_text, "")

    def test_chunk_text_basic(self):
        text = "This is a sample text to be chunked into smaller pieces."
        chunks = chunk_text(text, chunk_size=10, overlap=0)
        self.assertEqual(chunks, ["This is a ", "sample tex", "t to be c", "hunked int", "o smaller ", "pieces."])

    def test_chunk_text_with_overlap(self):
        text = "This is a sample text to be chunked with overlap."
        chunks = chunk_text(text, chunk_size=10, overlap=5)
        self.assertEqual(chunks, ["This is a ", "ample text", "text to be", "o be chun", "chunked wi", "ed with ov", "ith overla", "verlap."])

    def test_chunk_text_empty(self):
        text = ""
        chunks = chunk_text(text, chunk_size=10, overlap=5)
        self.assertEqual(chunks, [])

    def test_chunk_text_short_text(self):
        text = "short"
        chunks = chunk_text(text, chunk_size=10, overlap=5)
        self.assertEqual(chunks, ["short"])

    def test_chunk_text_overlap_greater_than_chunk_size(self):
        text = "some text"
        chunks = chunk_text(text, chunk_size=5, overlap=10) # Should not cause infinite loop
        self.assertEqual(chunks, ["some "]) # The loop should break after the first chunk

    @patch('backend.main.co', new_callable=MockCohereClient)
    @patch.dict(os.environ, {'COHERE_API_KEY': 'mock_api_key'})
    def test_embed_integration(self, mock_cohere_client):
        chunks = ["hello world", "this is a test"]
        embeddings = embed(chunks)
        
        self.assertEqual(len(embeddings), 2)
        self.assertEqual(len(embeddings[0]), 1024) # Assuming Cohere embedding size
        self.assertTrue(all(isinstance(val, float) for val in embeddings[0]))
        # Verify that the mock client's embed method was called
        mock_cohere_client.embed.assert_called_once_with(
            texts=chunks,
            model="embed-english-v3.0",
            input_type="classification"
        )
    
    @patch('backend.main.co', new_callable=MockCohereClient)
    @patch.dict(os.environ, {'COHERE_API_KEY': ''}) # Simulate missing API key
    def test_embed_no_api_key_handling(self, mock_cohere_client):
        chunks = ["test chunk"]
        with self.assertRaises(cohere.CohereError): # Expecting an error from the mock client
            embed(chunks)

    @patch('backend.main.qdrant_client', new_callable=MockQdrantClient)
    @patch.dict(os.environ, {'QDRANT_API_KEY': 'mock_qdrant_api_key', 'QDRANT_URL': 'mock_url'})
    def test_create_collection(self, mock_qdrant_client):
        collection_name = "test_collection"
        vector_size = 1024
        
        create_collection(collection_name, vector_size)
        
        self.assertIn(collection_name, mock_qdrant_client.collections)
        self.assertEqual(mock_qdrant_client.collections[collection_name]["vectors_config"].size, vector_size)
        self.assertEqual(mock_qdrant_client.collections[collection_name]["vectors_config"].distance, models.Distance.COSINE)
        # Verify that recreate_collection was called
        mock_qdrant_client.recreate_collection.assert_called_once_with(
            collection_name=collection_name,
            vectors_config=models.VectorParams(size=vector_size, distance=models.Distance.COSINE)
        )

    @patch('backend.main.qdrant_client', new_callable=MockQdrantClient)
    @patch.dict(os.environ, {'QDRANT_API_KEY': '', 'QDRANT_URL': 'mock_url'}) # Simulate missing API key
    def test_create_collection_no_api_key_handling(self, mock_qdrant_client):
        collection_name = "test_collection"
        vector_size = 1024
        with self.assertRaises(Exception): # Expecting an error from the mock client
            create_collection(collection_name, vector_size)

    @patch('backend.main.qdrant_client', new_callable=MockQdrantClient)
    @patch.dict(os.environ, {'QDRANT_API_KEY': 'mock_qdrant_api_key', 'QDRANT_URL': 'mock_url'})
    def test_save_chunk_to_qdrant(self, mock_qdrant_client):
        collection_name = "test_collection_save"
        chunks = ["chunk1", "chunk2"]
        embeddings = [[0.1]*1024, [0.2]*1024]
        metadata_list = [{"url": "url1", "title": "title1", "chunk_index": 0}, {"url": "url2", "title": "title2", "chunk_index": 1}]

        # Ensure collection exists in mock client
        mock_qdrant_client.collections[collection_name] = {
            "vectors_config": models.VectorParams(size=1024, distance=models.Distance.COSINE),
            "points": []
        }
        
        save_chunk_to_qdrant(collection_name, chunks, embeddings, metadata_list)
        
        self.assertEqual(len(mock_qdrant_client.collections[collection_name]["points"]), 2)
        
        # Verify content of the first point
        first_point = mock_qdrant_client.collections[collection_name]["points"][0]
        self.assertEqual(first_point.vector, embeddings[0])
        self.assertEqual(first_point.payload["text_chunk"], chunks[0])
        self.assertEqual(first_point.payload["url"], metadata_list[0]["url"])
        self.assertEqual(first_point.payload["title"], metadata_list[0]["title"])
        
        # Verify that upsert was called
        mock_qdrant_client.upsert.assert_called_once()
    
    @patch('backend.main.qdrant_client', new_callable=MockQdrantClient)
    @patch.dict(os.environ, {'QDRANT_API_KEY': 'mock_qdrant_api_key', 'QDRANT_URL': 'mock_url'})
    def test_save_chunk_to_qdrant_no_collection(self, mock_qdrant_client):
        collection_name = "non_existent_collection"
        chunks = ["chunk1"]
        embeddings = [[0.1]*1024]
        metadata_list = [{"url": "url1", "title": "title1", "chunk_index": 0}]

        with self.assertRaises(Exception): # Expecting an error as collection does not exist
            save_chunk_to_qdrant(collection_name, chunks, embeddings, metadata_list)

if __name__ == '__main__':
    unittest.main()
