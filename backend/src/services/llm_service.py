import logging
import json
import re
import google.generativeai as genai
from src.core.settings import settings
from src.services.embedding_service import generate_embedding
from src.services.qdrant_service import QdrantService

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class LLMService:
    def __init__(self):
        # Configure Gemini
        genai.configure(api_key=settings.GEMINI_API_KEY)
        self.model = genai.GenerativeModel('gemini-2.5-flash')
        self.qdrant_service = QdrantService()

    async def get_response(self, messages: list, selected_text: str = None) -> str:
        """
        Handles the conversation with Gemini, using RAG for book content retrieval.
        """
        
        # Get the user's question from the last message
        user_question = messages[-1]["content"] if messages else ""
        
        # Dynamic, soft-coded system prompt that guides conversational behavior
        system_prompt = """You are a friendly and knowledgeable assistant for a book about Physical AI and Humanoid Robotics. 

Your personality:
- Warm, approachable, and genuinely helpful
- Speak naturally like a knowledgeable friend, not a robot
- Use conversational language with contractions (don't, can't, I'll, etc.)
- Be enthusiastic when explaining exciting concepts

Your behavior:
- Base your answers ONLY on the provided context from the book - never make up facts
- If the context doesn't contain relevant information, honestly say you couldn't find that specific topic in the book
- When given selected text as context, focus your answer specifically on that section
- Connect concepts naturally and explain them in an accessible way
- Keep responses concise but complete - don't ramble

Response style:
- Start with a direct answer, then elaborate if helpful
- Use examples from the book when available
- Break down complex topics into digestible pieces
- End with a brief invitation for follow-up questions when appropriate"""
        try:
            # Pattern matching to detect specific chapter requests
            # e.g., "summarize module 1 chapter 4" or "Module 1 Chapter 2 summary"
            module_match = re.search(r'module\s*(\d+)', user_question, re.IGNORECASE)
            chapter_match = re.search(r'chapter\s*(\d+)', user_question, re.IGNORECASE)
            
            module_id = f"module-{module_match.group(1)}" if module_match else None
            chapter_id = f"chapter-{chapter_match.group(1)}" if chapter_match else None
            
            logger.info(f"Searching book for: '{user_question}'. Detected filters: module={module_id}, chapter={chapter_id}")
            
            if module_id and chapter_id:
                # If both are specified, try to get all chunks for that chapter for a full overview
                chunks_data = self.qdrant_service.get_all_chapter_chunks(module_id, chapter_id)
                # Fallback to similarity if no exact match found or if user asked for a specific sub-topic
                if not chunks_data:
                    query_vector = generate_embedding(user_question)
                    chunks_data = self.qdrant_service.retrieve_relevant_chunks(query_vector, top_k=20, module_filter=module_id, chapter_filter=chapter_id)
            else:
                # General search with higher top_k
                query_vector = generate_embedding(user_question)
                chunks_data = self.qdrant_service.retrieve_relevant_chunks(query_vector, top_k=15, module_filter=module_id)
            
            # Format the retrieved context
            book_context = "\n\n".join([
                f"[Source: {c.get('metadata', {}).get('title', 'Unknown Section')} - {c.get('metadata', {}).get('module', '')} {c.get('metadata', {}).get('chapter', '')}]\n{c.get('text', '')}" 
                for c in chunks_data
            ])
            
            if not book_context.strip():
                book_context = "No relevant content found in the book for this query."
            
            # Build the prompt with context
            if selected_text:
                context_section = f"""
## User's Selected Text
The user has highlighted this specific text from the book:
\"\"\"{selected_text}\"\"\"

Focus your answer on this selected content."""
            else:
                context_section = ""

            full_prompt = f"""{system_prompt}
{context_section}

## Retrieved Book Content
{book_context}

## User Question
{user_question}

Please answer based on the book content above. If the user asked for a summary, provide a comprehensive one based on the retrieved content."""

            # Call Gemini
            response = self.model.generate_content(full_prompt)
            
            return response.text

        except Exception as e:
            logger.error(f"Error in LLMService: {e}", exc_info=True)
            return "I'm sorry, I encountered an error while processing your request. Please try again."
