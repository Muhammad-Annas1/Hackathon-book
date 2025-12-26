import logging
from fastapi import APIRouter, HTTPException, BackgroundTasks
from src.api.v1.schemas import ChatRequest, ChatResponse
from src.services.llm_service import LLMService
from src.services.postgres_service import PostgresService

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

router = APIRouter()

# Initialize services
llm_service = LLMService()
postgres_service = PostgresService()

@router.post("/chat", response_model=ChatResponse)
async def handle_chat_request(request: ChatRequest, background_tasks: BackgroundTasks):
    """
    Handles a chat request using the soft-coded agent approach.
    Logs conversations to Neon Postgres in the background.
    """
    logger.info(f"Received chat request: '{request.question}'")
    try:
        # Construct message history (stateless for now, just current turn)
        messages = [{"role": "user", "content": request.question}]
        
        # Call Agent
        answer = await llm_service.get_response(
            messages=messages,
            selected_text=request.context 
        )
        
        # Log chat to Postgres in background (non-blocking)
        background_tasks.add_task(
            postgres_service.log_chat,
            request.question,
            answer
        )
        
        return ChatResponse(
            answer=answer,
            source_nodes=[]  # Sources are handled internally by the agent
        )

    except Exception as e:
        logger.error(f"An error occurred in the chat endpoint: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail="An internal server error occurred.")
