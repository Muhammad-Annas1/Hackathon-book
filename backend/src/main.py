from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from src.api.v1 import chat

# Create the FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    description="API for the RAG (Retrieval-Augmented Generation) Chatbot.",
    version="1.0.0",
)

# Configure CORS (Cross-Origin Resource Sharing)
# This allows the Docusaurus frontend (running on a different port) to communicate with the backend.
origins = [
    "http://localhost",
    "http://localhost:3000",  # Default Docusaurus dev server port
    "https://hackathon-book-cyan.vercel.app",  # Production Vercel URL
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"], # Allow all methods
    allow_headers=["*"], # Allow all headers
)

# Include the chat API router
# All routes defined in src.api.v1.chat will be prefixed with /api/v1
app.include_router(chat.router, prefix="/api/v1", tags=["Chat"])

@app.get("/", tags=["Root"])
async def read_root():
    """
    A simple health check endpoint.
    """
    return {"status": "ok", "message": "Welcome to the RAG Chatbot API!"}
