# Quickstart Guide: RAG Chatbot Backend

This guide provides the steps to set up and run the FastAPI backend for the RAG chatbot.

## Prerequisites

- Python 3.11+
- `uv` installed (`pip install uv`)
- Access to OpenRouter, Gemini, and Qdrant Cloud to get API keys.

## 1. Setup Backend Project

Navigate to the repository root and create the backend directory structure. This plan will be executed by the agent, but the manual steps are documented here for clarity.

```bash
# Create the main backend folder
mkdir backend
cd backend

# Initialize the uv project and virtual environment
uv venv
uv pip install -e . 
```

The agent will create the file structure as defined in `plan.md`, including `pyproject.toml`, `src/main.py`, etc.

## 2. Configure Environment Variables

Create a `.env` file in the `backend/` directory by copying the example file.

```bash
cp .env.example .env
```

Now, edit the `.env` file and add your credentials:

```ini
# .env
OPENROUTER_API_KEY="sk-or-..."
GEMINI_API_KEY="AIzaSy..."
QDRANT_URL="..."
QDRANT_API_KEY="..."
```

## 3. Install Dependencies

With your virtual environment activated (uv automatically activates it in the current shell), install the project dependencies.

```bash
# Uv will read the pyproject.toml and install dependencies
uv pip install -e . 
```

## 4. Run the Backend Server

Launch the FastAPI server using `uv`.

```bash
uv run --host 0.0.0.0 --port 8000 src.main:app
```

The API will now be available at `http://localhost:8000`.

## 5. Connecting Frontend to Backend

The Docusaurus frontend will need to be configured to make requests to this backend. The chat component will use the `fetch` API to send POST requests to `http://localhost:8000/api/v1/chat`.

**Example Frontend Request:**

```javascript
async function getChatbotResponse(question, context = null) {
  const response = await fetch('http://localhost:8000/api/v1/chat', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify({ question, context }),
  });

  if (!response.ok) {
    throw new Error('Network response was not ok');
  }

  return response.json();
}
```

This covers the local connection plan between the frontend and backend. CORS will be enabled in the FastAPI application to allow requests from the Docusaurus development server.
