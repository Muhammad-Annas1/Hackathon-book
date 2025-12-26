import os
import requests
from bs4 import BeautifulSoup
from urllib.parse import urljoin, urlparse
from dotenv import load_dotenv
from qdrant_client import QdrantClient, models
from qdrant_client.http.models import PointStruct
import uuid
import logging
from typing import List
import xml.etree.ElementTree as ElementTree
from sentence_transformers import SentenceTransformer

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# Load environment variables
load_dotenv()

QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")

if not QDRANT_API_KEY or not QDRANT_URL:
    logging.error("QDRANT_API_KEY or QDRANT_URL not found in environment variables.")
    # We might want to raise an error here in production, but for now we'll let it fail later if needed or just log
    # exit(1) 

# Initialize Qdrant client (Use local persistence for reliability in dev environment)
try:
    if QDRANT_URL and "localhost" not in QDRANT_URL:
        # Try Cloud
        try:
             qdrant_client = QdrantClient(
                url=QDRANT_URL,
                api_key=QDRANT_API_KEY,
            )
             qdrant_client.get_collections() # Test connection
        except:
             logging.warning("Failed to connect to Qdrant Cloud. Falling back to local storage.")
             qdrant_client = QdrantClient(path="./qdrant_data")
    else:
        qdrant_client = QdrantClient(path="./qdrant_data")
        
except Exception as e:
    logging.error(f"Failed to initialize Qdrant client: {e}")
    exit(1)

# Initialize SentenceTransformer model (Local Embeddings)
# "all-MiniLM-L6-v2" maps sentences & paragraphs to a 384 dimensional dense vector space.
try:
    model = SentenceTransformer('all-MiniLM-L6-v2')
except Exception as e:
    logging.error(f"Failed to load SentenceTransformer model: {e}")
    exit(1)


def get_urls_from_sitemap(sitemap_url: str) -> List[str]:
    logging.info(f"Fetching URLs from sitemap: {sitemap_url}")
    try:
        response = requests.get(sitemap_url, timeout=10)
        response.raise_for_status()
        
        root = ElementTree.fromstring(response.content)
        
        urls = []
        # Check for <sitemapindex> for nested sitemaps
        sitemap_namespace = {'s': 'http://www.sitemaps.org/schemas/sitemap/0.9'}
        for sitemap_elem in root.findall('s:sitemap', sitemap_namespace):
            nested_sitemap_url = sitemap_elem.find('s:loc', sitemap_namespace).text
            urls.extend(get_urls_from_sitemap(nested_sitemap_url))
        
        # Extract URLs from <urlset>
        urlset_namespace = {'u': 'http://www.sitemaps.org/schemas/sitemap/0.9'} 
        for url_elem in root.findall('u:url', urlset_namespace):
            loc_elem = url_elem.find('u:loc', urlset_namespace)
            if loc_elem is not None:
                urls.append(loc_elem.text)
        
        logging.info(f"Found {len(urls)} URLs in sitemap {sitemap_url}")
        return urls
    except Exception as e:
        logging.error(f"Error processing sitemap {sitemap_url}: {e}")
        return []

def extract_text_from_url(url: str) -> str:
    logging.info(f"Extracting text from {url}")
    try:
        response = requests.get(url, timeout=5)
        response.raise_for_status()
    except Exception as e:
        logging.warning(f"Could not fetch {url}: {e}")
        return ""

    soup = BeautifulSoup(response.text, 'html.parser')

    # Remove unwanted elements
    for script_or_style in soup(['script', 'style', 'header', 'footer', 'nav', 'aside']):
        script_or_style.decompose()

    # Try to find the main content container
    # Docusaurus usually uses <main> or specific classes
    main_content = soup.find('main') or soup.find('article') or soup.find('body')
    
    if not main_content:
        return ""

    text = main_content.get_text(separator=' ', strip=True)
    return text

def chunk_text(text: str, chunk_size: int = 512, overlap: int = 64) -> list[str]:
    if not text:
        return []
    chunks = []
    start = 0
    while start < len(text):
        end = start + chunk_size
        chunk = text[start:end]
        chunks.append(chunk)
        start += (chunk_size - overlap)
    return chunks

def create_collection(collection_name: str, vector_size: int = 384):
    logging.info(f"Creating Qdrant collection '{collection_name}' with vector size {vector_size}")
    try:
        qdrant_client.recreate_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(size=vector_size, distance=models.Distance.COSINE),
        )
        logging.info(f"Collection '{collection_name}' created successfully.")
    except Exception as e:
        logging.error(f"Failed to create collection '{collection_name}': {e}")
        raise

def extract_metadata_from_url(url: str) -> dict:
    """
    Extracts module and chapter information from a Docusaurus URL.
    Example: .../docs/module-1/introduction -> {module: 'module-1', chapter: 'introduction'}
    """
    path_parts = urlparse(url).path.split('/')
    # Remove empty strings
    parts = [p for p in path_parts if p]
    
    metadata = {
        "module": "general",
        "chapter": "general",
        "title": "Untitled"
    }
    
    # Typical Docusaurus structure: /docs/module-name/chapter-name
    if "docs" in parts:
        docs_index = parts.index("docs")
        if len(parts) > docs_index + 1:
            metadata["module"] = parts[docs_index + 1]
        if len(parts) > docs_index + 2:
            metadata["chapter"] = parts[docs_index + 2]
            
    return metadata

def process_and_index(urls: List[str], collection_name: str):
    points = []
    total_chunks = 0
    
    for url in urls:
        raw_text = extract_text_from_url(url)
        if not raw_text:
            continue
            
        url_metadata = extract_metadata_from_url(url)
        
        # Try to extract a title from the text (assuming first line or heading)
        # BeautifulSoup in extract_text_from_url already cleaned it up, but we can do better
        soup = BeautifulSoup(requests.get(url).text, 'html.parser')
        title_tag = soup.find('h1') or soup.find('title')
        page_title = title_tag.get_text().strip() if title_tag else url_metadata["chapter"].replace('-', ' ').title()
        
        chunks = chunk_text(raw_text)
        if not chunks:
            continue
            
        logging.info(f"Encoding {len(chunks)} chunks for {url} (Title: {page_title})")
        embeddings = model.encode(chunks).tolist()
        
        for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
            point_id = str(uuid.uuid4())
            payload = {
                "url": url,
                "text": chunk,
                "chunk_index": i,
                "module": url_metadata["module"],
                "chapter": url_metadata["chapter"],
                "title": page_title
            }
            points.append(PointStruct(
                id=point_id,
                vector=embedding,
                payload=payload
            ))
            total_chunks += 1

    if points:
        logging.info(f"Upserting {len(points)} points to Qdrant...")
        # Batch upsert can be optimized, but for this size it's fine
        batch_size = 100
        for i in range(0, len(points), batch_size):
            batch = points[i:i+batch_size]
            try:
                qdrant_client.upsert(
                    collection_name=collection_name,
                    points=batch
                )
            except Exception as e:
                logging.error(f"Error upserting batch: {e}")
                
        logging.info(f"Ingestion complete. Total chunks: {total_chunks}")
    else:
        logging.warning("No data found to ingest.")

def main():
    sitemap_url = "https://hackathon-book-cyan.vercel.app/sitemap.xml" 
    # Or local URL if running locally: "http://localhost:3000/sitemap.xml"
    # But usually parsing the live site is easier if it exists, or we can crawl local files.
    # Given the previous main.py used the vercel app, we will stick to that or asking user.
    # For now, let's use the hardcoded one from previous file or ENV.
    
    collection_name = "rag_embedding"
    
    logging.info("Starting ingestion...")
    
    urls = get_urls_from_sitemap(sitemap_url)
    
    # Correct URLs if they use the default Docusaurus domain
    old_domain = "https://your-docusaurus-test-site.com"
    target_base = "https://hackathon-book-cyan.vercel.app/"
    
    corrected_urls = []
    for url in urls:
        if url.startswith(old_domain):
            corrected_urls.append(url.replace(old_domain, target_base.rstrip('/')))
        else:
            corrected_urls.append(url)
            
    # Filter for only relevant pages
    urls = [u for u in corrected_urls if u.startswith(target_base)]
    
    if not urls:
        logging.error("No URLs found. Check sitemap URL.")
        return

    # Create collection (384 dim for all-MiniLM-L6-v2)
    create_collection(collection_name, vector_size=384)
    
    process_and_index(urls, collection_name)

if __name__ == "__main__":
    main()
