# RAG Chatbot Backend

This directory contains the FastAPI backend for the RAG (Retrieval-Augmented Generation) chatbot.

## Overview

The backend provides a simple API endpoint (`/api/v1/chat`) that accepts a user's question, retrieves relevant context from a Qdrant vector database, and uses an LLM via OpenRouter to generate a factually-grounded answer.

## Tech Stack

- **Python 3.11+**
- **FastAPI**: For the web framework.
- **uv**: For project and dependency management.
- **Pydantic**: For data validation and settings management.
- **Qdrant**: For vector storage and retrieval.
- **OpenRouter**: For accessing various Large Language Models.
- **Gemini**: For generating text embeddings.

## Getting Started

### Prerequisites

- Python 3.11+
- `uv` installed (`pip install uv`)
- API keys for OpenRouter, Gemini, and Qdrant.

### 1. Setup the Environment

First, create a virtual environment using `uv`.

```bash
# From the /backend directory
uv venv
```

### 2. Configure Credentials

Copy the example environment file and fill in your credentials.

```bash
cp .env.example .env
```

Edit the `.env` file with your actual API keys and Qdrant URL.

### 3. Install Dependencies

Install all required Python packages.

```bash
# This command reads the pyproject.toml and installs dependencies
uv pip install -e .
```

### 4. Run the Server

Launch the FastAPI application using `uvicorn` (which is installed as a dependency of `fastapi`).

```bash
# From the /backend directory
uvicorn src.main:app --host 0.0.0.0 --port 8000 --reload
```

- The `--reload` flag enables hot-reloading, which is useful for development.
- The API will be available at `http://localhost:8000`.
- The OpenAPI documentation (Swagger UI) will be at `http://localhost:8000/docs`.

## Project Structure

- `src/main.py`: The main FastAPI application entrypoint.
- `src/api/v1/`: Contains the API endpoint logic.
  - `chat.py`: The `/chat` endpoint.
  - `schemas.py`: Pydantic models for request/response validation.
- `src/core/`: Core application logic.
  - `settings.py`: Environment variable and settings management.
- `src/services/`: Business logic and external service integrations.
  - `embedding_service.py`: Handles text embedding generation via Gemini.
  - `qdrant_service.py`: Handles context retrieval from Qdrant.
  - `llm_service.py`: Handles interaction with the LLM via OpenRouter.
