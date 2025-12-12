# Quickstart for RAG Ingestion Pipeline

This guide explains how to set up and run the RAG ingestion pipeline.

## 1. Prerequisites

- Python 3.11 or higher
- `uv` package manager installed
- A Qdrant Cloud account and API key
- A Cohere account and API key

## 2. Setup

1.  **Clone the repository**:
    ```bash
    git clone <repository-url>
    cd <repository-directory>
    ```

2.  **Create a virtual environment**:
    ```bash
    python -m venv .venv
    source .venv/bin/activate
    ```

3.  **Install dependencies**:
    ```bash
    uv pip install -r requirements.txt
    ```

4.  **Set environment variables**:
    Create a `.env` file in the `backend` directory with the following content:
    ```
    COHERE_API_KEY=<your-cohere-api-key>
    QDRANT_API_KEY=<your-qdrant-api-key>
    QDRANT_URL=<your-qdrant-cloud-url>
    ```

## 3. Running the script

1.  **Navigate to the backend directory**:
    ```bash
    cd backend
    ```

2.  **Run the script**:
    ```bash
    python main.py
    ```

The script will then:
1.  Fetch all URLs from the Docusaurus site.
2.  Extract the text from each URL.
3.  Chunk the text.
4.  Generate embeddings for each chunk.
5.  Create a new Qdrant collection.
6.  Save the chunks and their embeddings to the collection.
