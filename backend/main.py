import os
import requests
from bs4 import BeautifulSoup
from urllib.parse import urljoin, urlparse
from dotenv import load_dotenv
import cohere # Import the cohere client
from qdrant_client import QdrantClient, models # Import Qdrant client
from qdrant_client.http.models import PointStruct # For PointStruct
import uuid
import logging

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# Load environment variables from .env file
load_dotenv()

COHERE_API_KEY = os.getenv("COHERE_API_KEY")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")

# Validate environment variables
if not COHERE_API_KEY:
    logging.error("COHERE_API_KEY not found in environment variables.")
    exit(1)
if not QDRANT_API_KEY or not QDRANT_URL:
    logging.error("QDRANT_API_KEY or QDRANT_URL not found in environment variables.")
    exit(1)

# Initialize Cohere client
try:
    co = cohere.Client(COHERE_API_KEY)
except cohere.CohereError as e:
    logging.error(f"Failed to initialize Cohere client: {e}")
    exit(1)

# Initialize Qdrant client
try:
    qdrant_client = QdrantClient(
        url=QDRANT_URL,
        api_key=QDRANT_API_KEY,
    )
except Exception as e:
    logging.error(f"Failed to initialize Qdrant client: {e}")
    exit(1)

def get_all_urls(base_url: str) -> list[str]:
    """
    Recursively finds all relevant URLs within the Docusaurus site, starting from the base_url.
    It crawls the site to discover internal links.
    """
    logging.info(f"Starting URL discovery from: {base_url}")
    visited_urls = set()
    urls_to_visit = [base_url]
    internal_urls = set()
    
    base_netloc = urlparse(base_url).netloc

    while urls_to_visit:
        current_url = urls_to_visit.pop(0)
        if current_url in visited_urls:
            continue
        
        logging.info(f"Visiting {current_url}")
        visited_urls.add(current_url)
        internal_urls.add(current_url)
        
        try:
            response = requests.get(current_url, timeout=5)
            response.raise_for_status()
        except requests.exceptions.RequestException as e:
            logging.warning(f"Could not fetch {current_url}: {e}")
            continue

        soup = BeautifulSoup(response.text, 'html.parser')
        
        for link in soup.find_all('a', href=True):
            href = link['href']
            resolved_url = urljoin(current_url, href)
            parsed_resolved_url = urlparse(resolved_url)
            
            if parsed_resolved_url.netloc == base_netloc and resolved_url not in visited_urls:
                if parsed_resolved_url.fragment and resolved_url.split('#')[0] == current_url.split('#')[0]:
                    continue
                urls_to_visit.append(resolved_url)

    logging.info(f"Finished URL discovery. Found {len(internal_urls)} internal URLs.")
    return list(internal_urls)

def extract_text_from_url(url: str) -> str:
    """
    Fetches HTML content from a given URL and extracts clean, readable text.
    It focuses on main content areas to avoid boilerplate text.
    """
    logging.info(f"Extracting text from {url}")
    try:
        response = requests.get(url, timeout=5)
        response.raise_for_status()
    except requests.exceptions.RequestException as e:
        logging.warning(f"Could not fetch {url}: {e}")
        return ""

    soup = BeautifulSoup(response.text, 'html.parser')

    for script_or_style in soup(['script', 'style', 'header', 'footer', 'nav', 'aside']):
        script_or_style.decompose()

    main_content = soup.find('main')
    if not main_content:
        main_content = soup.find('body')

    text = main_content.get_text(separator=' ', strip=True)
    return text

def chunk_text(text: str, chunk_size: int, overlap: int) -> list[str]:
    """
    Chunks text into segments of a fixed size with some overlap.
    """
    if not text:
        return []

    chunks = []
    start = 0
    while start < len(text):
        end = start + chunk_size
        chunk = text[start:end]
        chunks.append(chunk)
        start += (chunk_size - overlap)
        if overlap > chunk_size:
            break 
    return chunks

def embed(chunks: list[str]) -> list[list[float]]:
    """Generates Cohere embeddings for a list of text chunks."""
    if not chunks:
        return []
    logging.info(f"Generating embeddings for {len(chunks)} chunks using Cohere")
    try:
        response = co.embed(
            texts=chunks,
            model="embed-english-v3.0",
            input_type="classification"
        )
        return [e for e in response.embeddings]
    except Exception as e:
        logging.error(f"Cohere embedding failed: {e}")
        return []

def create_collection(collection_name: str, vector_size: int):
    """
    Creates a new collection in Qdrant if it doesn't already exist.
    """
    logging.info(f"Creating Qdrant collection '{collection_name}' with vector size {vector_size}")
    try:
        qdrant_client.recreate_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(size=vector_size, distance=models.Distance.COSINE),
        )
        logging.info(f"Collection '{collection_name}' created or re-created successfully.")
    except Exception as e:
        logging.error(f"Failed to create or re-create Qdrant collection '{collection_name}': {e}")
        raise


def save_chunk_to_qdrant(collection_name: str, chunks: list[str], embeddings: list[list[float]], metadata_list: list[dict]):
    """
    Saves the chunks, their embeddings, and associated metadata to the Qdrant collection.
    """
    logging.info(f"Saving {len(chunks)} chunks to Qdrant collection '{collection_name}'")
    points = []
    for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
        metadata = metadata_list[i] if i < len(metadata_list) and isinstance(metadata_list[i], dict) else {}
        metadata['text_chunk'] = chunk # Store the actual text chunk in payload

        points.append(
            PointStruct(
                id=str(uuid.uuid4()),
                vector=embedding,
                payload=metadata,
            )
        )
    
    try:
        qdrant_client.upsert(
            collection_name=collection_name,
            points=points,
            wait=True,
        )
        logging.info(f"Successfully upserted {len(points)} points to collection '{collection_name}'.")
    except Exception as e:
        logging.error(f"Failed to upsert points to Qdrant collection '{collection_name}': {e}")
        raise

def main():
    logging.info("RAG Ingestion Pipeline started.")
    
    base_url = "https://hackathon-book-cyan.vercel.app/"
    collection_name = "rag_embedding"
    chunk_size = 512
    overlap = 64

    # 1. Content Extraction
    urls = get_all_urls(base_url)
    
    all_extracted_content = []
    for url in urls:
        text = extract_text_from_url(url)
        if text:
            all_extracted_content.append({"url": url, "text": text})
            logging.info(f"Extracted content from {url[:50]}... (len: {len(text)})")
        else:
            logging.warning(f"No content extracted from {url}")

    if not all_extracted_content:
        logging.error("No content extracted from the Docusaurus site. Exiting.")
        return

    logging.info(f"Total unique URLs found and processed: {len(all_extracted_content)}")
    logging.info("--- Content Extraction Complete ---")

    # 2. Embedding Generation and Qdrant Storage
    total_chunks_processed = 0
    for content_item in all_extracted_content:
        text_to_chunk = content_item["text"]
        chunks = chunk_text(text_to_chunk, chunk_size=chunk_size, overlap=overlap)
        logging.info(f"Chunked content from {content_item['url'][:50]}... into {len(chunks)} chunks.")
        
        embeddings = embed(chunks)
        if embeddings:
            logging.info(f"Generated embeddings for {len(embeddings)} chunks.")
            
            if not embeddings:
                logging.warning(f"No embeddings generated for {content_item['url'][:50]}..., skipping Qdrant save.")
                continue

            vector_size = len(embeddings[0]) 
            try:
                create_collection(collection_name, vector_size) # Create/recreate once
            except Exception as e:
                logging.error(f"Pipeline stopped due to Qdrant collection error: {e}")
                return # Exit if collection creation fails
            
            metadata_list = []
            for i, chunk in enumerate(chunks):
                metadata = {
                    "url": content_item["url"],
                    "title": content_item.get("title", "Untitled"),
                    "chunk_index": i
                }
                metadata_list.append(metadata)
            
            try:
                save_chunk_to_qdrant(collection_name, chunks, embeddings, metadata_list)
                total_chunks_processed += len(chunks)
            except Exception as e:
                logging.error(f"Failed to save chunks from {content_item['url'][:50]}... to Qdrant: {e}")
        else:
            logging.warning(f"No embeddings generated for {content_item['url'][:50]}..., skipping Qdrant save.")
    
    logging.info(f"RAG Ingestion Pipeline finished. Total chunks processed and saved to Qdrant: {total_chunks_processed}")

if __name__ == "__main__":
    main()