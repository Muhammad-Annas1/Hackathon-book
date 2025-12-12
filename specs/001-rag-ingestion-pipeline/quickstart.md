# Quickstart for RAG Ingestion Pipeline

This guide explains how to set up and run the RAG ingestion pipeline.

## 1. Prerequisites

- Python 3.11 or higher
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
    ```
    Activate the virtual environment:
    ```bash
    # On Windows
    .venv\Scripts\activate
    # On macOS/Linux
    source .venv/bin/activate
    ```

3.  **Install `uv` and project dependencies**:
    ```bash
    python -m pip install uv
    uv pip install -r backend/requirements.txt
    ```
    (Note: If `uv` is already installed globally, you might just need `uv pip install -r backend/requirements.txt`)

4.  **Set environment variables**:
    Create a `.env` file in the **project root** directory with the following content:
    ```
    COHERE_API_KEY=<your-cohere-api-key>
    QDRANT_API_KEY=<your-qdrant-api-key>
    QDRANT_URL=<your-qdrant-cloud-url>
    ```

## 3. Running the script

1.  **Ensure virtual environment is activated** (see step 2).

2.  **Run the script from the project root**:
    ```bash
    python -m backend.main
    ```

The script will then:
1.  Fetch all URLs from the Docusaurus site.
2.  Extract the text from each URL.
3.  Chunk the text.
4.  Generate embeddings for each chunk.
5.  Create a new Qdrant collection (or recreate if exists).
6.  Save the chunks and their embeddings to the collection.