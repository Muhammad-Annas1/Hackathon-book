from qdrant_client import QdrantClient
from src.core.settings import settings
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def verify_qdrant_data():
    try:
        # Initialize client
        if settings.QDRANT_URL and "localhost" not in settings.QDRANT_URL:
            client = QdrantClient(
                url=settings.QDRANT_URL, 
                api_key=settings.QDRANT_API_KEY,
            )
            print(f"Connected to Qdrant Cloud: {settings.QDRANT_URL}")
        else:
            client = QdrantClient(path="./qdrant_data")
            print("Connected to Local Qdrant")

        collection_name = "rag_embedding"
        
        # Get collection info
        collection_info = client.get_collection(collection_name)
        print(f"\nCollection '{collection_name}' Status: {collection_info.status}")
        print(f"Total Vectors: {collection_info.points_count}")

        # Fetch one point as a sample
        print("\nFetching a sample data point...")
        sample_points, _ = client.scroll(
            collection_name=collection_name,
            limit=1,
            with_payload=True,
            with_vectors=False
        )

        if sample_points:
            point = sample_points[0]
            print(f"Sample Point ID: {point.id}")
            print(f"Metadata (Payload):")
            for key, value in point.payload.items():
                if key != "text": # Skip printing the full text content for brevity
                    print(f"  - {key}: {value}")
            print(f"  - text: {point.payload.get('text', '')[:100]}... (truncated)")
        else:
            print("No points found in the collection.")

    except Exception as e:
        print(f"Error checking Qdrant data: {e}")

if __name__ == "__main__":
    verify_qdrant_data()
