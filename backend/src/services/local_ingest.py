import os
import uuid
import logging
import yaml
import re
from typing import List, Dict
from dotenv import load_dotenv
from qdrant_client import QdrantClient, models
from qdrant_client.http.models import PointStruct
from sentence_transformers import SentenceTransformer
import pathlib

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Load environment variables
load_dotenv()

QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
COLLECTION_NAME = "rag_embedding"

# Initialize Qdrant client
try:
    if QDRANT_URL and "localhost" not in QDRANT_URL:
        qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
    else:
        qdrant_client = QdrantClient(path="./qdrant_data")
except Exception as e:
    logger.error(f"Failed to initialize Qdrant client: {e}")
    exit(1)

# Initialize SentenceTransformer
try:
    model = SentenceTransformer('all-MiniLM-L6-v2')
except Exception as e:
    logger.error(f"Failed to load embedding model: {e}")
    exit(1)

def parse_markdown_file(file_path: str) -> Dict:
    """Parses a markdown file to extract frontmatter and content."""
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Simple frontmatter parser
    if content.startswith('---'):
        parts = content.split('---', 2)
        if len(parts) >= 3:
            try:
                metadata = yaml.safe_load(parts[1])
                body = parts[2].strip()
                return {"metadata": metadata, "body": body}
            except Exception as e:
                logger.warning(f"Failed to parse frontmatter in {file_path}: {e}")
    
    return {"metadata": {}, "body": content.strip()}

def chunk_text(text: str, chunk_size: int = 800, overlap: int = 100) -> List[str]:
    """Chunks text into segments."""
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

def extract_structural_info(file_path: str):
    """Extracts module and chapter info from the file path."""
    path_obj = pathlib.Path(file_path)
    parts = path_obj.parts
    
    module_id = None
    chapter_id = None
    
    # Expecting path like docs/module-X/chapter-Y-...
    for part in parts:
        if part.startswith('module-'):
            # Standardize to module-N
            m = re.search(r'module-(\d+)', part, re.IGNORECASE)
            if m:
                module_id = f"module-{m.group(1)}"
            else:
                module_id = part
        elif part.startswith('chapter-'):
            # Standardize to chapter-N
            m = re.search(r'chapter-(\d+)', part, re.IGNORECASE)
            if m:
                chapter_id = f"chapter-{m.group(1)}"
            else:
                chapter_id = part.split('.')[0]
            
    return module_id, chapter_id

def process_docs(docs_dir: str):
    points = []
    
    for root, dirs, files in os.walk(docs_dir):
        for file in files:
            if file.endswith('.md'):
                file_path = os.path.join(root, file)
                logger.info(f"Processing {file_path}")
                
                doc_data = parse_markdown_file(file_path)
                module_id, chapter_id = extract_structural_info(file_path)
                
                title = doc_data['metadata'].get('title', file.replace('.md', '').replace('-', ' ').title())
                body = doc_data['body']
                
                chunks = chunk_text(body)
                if not chunks:
                    continue
                
                embeddings = model.encode(chunks).tolist()
                
                for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
                    point_id = str(uuid.uuid4())
                    points.append(PointStruct(
                        id=point_id,
                        vector=embedding,
                        payload={
                            "text": chunk,
                            "title": title,
                            "module": module_id,
                            "chapter": chapter_id,
                            "source": file_path,
                            "chunk_index": i
                        }
                    ))

    return points

def main():
    docs_dir = os.path.abspath(os.path.join(os.getcwd(), "..", "docs"))
    if not os.path.exists(docs_dir):
        # Try relative to backend
        docs_dir = os.path.abspath(os.path.join(os.getcwd(), "docs"))
        if not os.path.exists(docs_dir):
            # Try parent then docs if we are in backend dir
            docs_dir = os.path.abspath("../docs")

    logger.info(f"Using docs directory: {docs_dir}")
    
    # Recreate collection to ensure clean state and correct vector size
    qdrant_client.recreate_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=models.VectorParams(size=384, distance=models.Distance.COSINE),
    )
    
    # Create payload indexes for structural filtering
    logger.info("Creating payload indexes for 'module' and 'chapter'...")
    qdrant_client.create_payload_index(
        collection_name=COLLECTION_NAME,
        field_name="module",
        field_schema=models.PayloadSchemaType.KEYWORD,
    )
    qdrant_client.create_payload_index(
        collection_name=COLLECTION_NAME,
        field_name="chapter",
        field_schema=models.PayloadSchemaType.KEYWORD,
    )
    
    points = process_docs(docs_dir)
    
    if points:
        logger.info(f"Upserting {len(points)} points to Qdrant...")
        batch_size = 50
        for i in range(0, len(points), batch_size):
            batch = points[i:i+batch_size]
            qdrant_client.upsert(
                collection_name=COLLECTION_NAME,
                points=batch
            )
        logger.info("Ingestion complete!")
    else:
        logger.warning("No documentation found to ingest.")

if __name__ == "__main__":
    main()
