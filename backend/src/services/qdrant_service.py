import logging
from qdrant_client import QdrantClient, models
from src.core.settings import settings

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class QdrantService:
    def __init__(self, collection_name: str = "rag_embedding"):
        """
        Initializes the Qdrant client.
        """
        logger.info(f"Initializing QdrantService for collection: '{collection_name}'")
        try:
            # Try connecting to Cloud first if configured
            if settings.QDRANT_URL and "localhost" not in settings.QDRANT_URL:
                try:
                    self.client = QdrantClient(
                        url=settings.QDRANT_URL, 
                        api_key=settings.QDRANT_API_KEY,
                    )
                    self.client.get_collections() # Test connection
                except Exception as e:
                    logger.warning(f"Could not connect to Qdrant Cloud: {e}. Using local storage.")
                    self.client = QdrantClient(path="./qdrant_data")
            else:
                 self.client = QdrantClient(path="./qdrant_data")

            self.collection_name = collection_name
            logger.info("Qdrant client initialized successfully.")
        except Exception as e:
            logger.error(f"Failed to initialize Qdrant client: {e}", exc_info=True)
            raise

    def retrieve_relevant_chunks(
        self, 
        query_vector: list[float], 
        top_k: int = 5, 
        module_filter: str = None, 
        chapter_filter: str = None
    ) -> list[dict]:
        """
        Retrieves the most relevant document chunks from Qdrant for a given query vector.
        Supports filtering by module and chapter.
        """
        logger.info(f"Retrieving top {top_k} chunks from collection '{self.collection_name}'. Filters: module={module_filter}, chapter={chapter_filter}")
        try:
            filter_conditions = []
            if module_filter:
                filter_conditions.append(models.FieldCondition(
                    key="module",
                    match=models.MatchValue(value=module_filter)
                ))
            if chapter_filter:
                filter_conditions.append(models.FieldCondition(
                    key="chapter",
                    match=models.MatchValue(value=chapter_filter)
                ))
            
            query_filter = models.Filter(must=filter_conditions) if filter_conditions else None

            # Use query_points which is the modern API in qdrant-client
            search_result = self.client.query_points(
                collection_name=self.collection_name,
                query=query_vector,
                limit=top_k,
                query_filter=query_filter,
                with_payload=True
            ).points
            
            logger.info(f"Qdrant search returned {len(search_result)} results.")
            
            retrieved_chunks = [
                {
                    "id": point.id,
                    "text": point.payload.get("text") or point.payload.get("text_chunk", ""),
                    "score": getattr(point, 'score', 0.0), # query_points might return different score attribute
                    "metadata": {k: v for k, v in point.payload.items() if k not in ("text", "text_chunk")}
                }
                for point in search_result
            ]
            logger.info(f"Successfully retrieved {len(retrieved_chunks)} chunks.")
            return retrieved_chunks

        except Exception as e:
            logger.error(f"An error occurred during Qdrant retrieval: {e}", exc_info=True)
            return []

    def get_all_chapter_chunks(self, module_id: str, chapter_id: str) -> list[dict]:
        """Fetches all chunks for a specific chapter regardless of vector similarity."""
        logger.info(f"Fetching all chunks for {module_id} {chapter_id}")
        try:
            scroll_result, _ = self.client.scroll(
                collection_name=self.collection_name,
                scroll_filter=models.Filter(
                    must=[
                        models.FieldCondition(key="module", match=models.MatchValue(value=module_id)),
                        models.FieldCondition(key="chapter", match=models.MatchValue(value=chapter_id))
                    ]
                ),
                limit=100,
                with_payload=True
            )
            
            return [
                {
                    "text": p.payload.get("text", ""),
                    "metadata": {k: v for k, v in p.payload.items() if k != "text"}
                }
                for p in scroll_result
            ]
        except Exception as e:
            logger.error(f"Error scrolling for chapter chunks: {e}")
            return []

# Example of how you might use this service
if __name__ == '__main__':
    # This is for demonstration purposes. In the actual app, this will be called from the API layer.
    
    # 1. Initialize the service
    qdrant_service = QdrantService()

    # 2. Get a sample embedding (replace with a real embedding from your embedding_service)
    # The dimension of this vector must match the dimension of the vectors in your Qdrant collection.
    # For Gemini's embedding-001, the dimension is 768.
    sample_query_vector = [0.1] * 768 # Replace with a real query vector

    # 3. Retrieve chunks
    relevant_chunks = qdrant_service.retrieve_relevant_chunks(sample_query_vector)

    # 4. Print results
    if relevant_chunks:
        print(f"Found {len(relevant_chunks)} relevant chunks:")
        for i, chunk in enumerate(relevant_chunks):
            print(f"  {i+1}. ID: {chunk['id']}, Score: {chunk['score']:.4f}")
            print(f"     Text: {chunk['text'][:100]}...") # Print first 100 characters
    else:
        print("No relevant chunks found.")
