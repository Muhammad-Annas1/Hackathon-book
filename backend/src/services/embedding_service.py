import logging
from sentence_transformers import SentenceTransformer

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize the model globally to avoid reloading (Singleton pattern via module level variable)
try:
    logger.info("Loading SentenceTransformer model 'all-MiniLM-L6-v2'...")
    model = SentenceTransformer('all-MiniLM-L6-v2')
    logger.info("SentenceTransformer model loaded successfully.")
except Exception as e:
    logger.error(f"Failed to load embedding model: {e}")
    model = None

def generate_embedding(text: str) -> list[float]:
    """
    Generates a vector embedding for a given text using the local SentenceTransformer model.

    Args:
        text: The input text to embed.

    Returns:
        A list of floats representing the vector embedding.
    """
    if model is None:
        raise RuntimeError("Embedding model is not initialized.")

    # logger.info(f"Generating embedding for text: {text[:50]}...")
    try:
        embedding = model.encode(text).tolist()
        return embedding
    except Exception as e:
        logger.error(f"An error occurred during embedding generation: {e}", exc_info=True)
        raise

async def abatch_generate_embedding(texts: list[str]) -> list[list[float]]:
    """
    Asynchronously generates embeddings for a batch of texts.
    (Note: SentenceTransformers is sync, so we run it in a thread or just call it directly since it's fast on CPU for small batches)
    """
    if model is None:
        raise RuntimeError("Embedding model is not initialized.")

    logger.info(f"Generating batch embeddings for {len(texts)} texts...")
    try:
        # Since model.encode is CPU bound, for high concurrency we might want run_in_executor, 
        # but for this scale direct call is likely fine or we can wrap it.
        import asyncio
        loop = asyncio.get_running_loop()
        embeddings = await loop.run_in_executor(None, lambda: model.encode(texts).tolist())
        return embeddings
    except Exception as e:
        logger.error(f"An error occurred during batch embedding generation: {e}", exc_info=True)
        raise
