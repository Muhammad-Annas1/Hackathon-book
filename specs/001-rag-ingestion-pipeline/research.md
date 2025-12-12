# Research for RAG Ingestion Pipeline

This document outlines the research and decisions made for the RAG ingestion pipeline.

## 1. Web Scraping with Requests and BeautifulSoup

- **Decision**: Use the `requests` library to fetch the HTML content of the URLs and `BeautifulSoup` to parse the HTML and extract the text.
- **Rationale**: This is a standard and robust combination for web scraping in Python. `requests` is simple for HTTP requests, and `BeautifulSoup` is excellent for navigating and searching the parse tree of an HTML document.
- **Alternatives considered**:
    - **Scrapy**: A more powerful web scraping framework, but it's overkill for this project, which only needs to fetch content from a known set of pages.
    - **LXML**: A faster XML and HTML parsing library, but `BeautifulSoup` has a more user-friendly API for this task.

## 2. Text Chunking Strategy

- **Decision**: Chunk the text into segments of a fixed size (e.g., 512 tokens) with some overlap (e.g., 64 tokens).
- **Rationale**: Fixed-size chunking is simple to implement and effective for many RAG applications. Overlap helps to preserve the context between chunks. The exact size will be determined by the Cohere model's context window and experimentation.
- **Alternatives considered**:
    - **Recursive chunking**: More complex to implement, but can be more effective at preserving semantic boundaries. For this project, fixed-size chunking is sufficient.
    - **Sentence-based chunking**: Can be effective, but long sentences can still exceed the context window.

## 3. Qdrant Collection Management

- **Decision**: Create a Qdrant collection with a specific vector size matching the Cohere embeddings. Use the `qdrant-client` library to interact with the Qdrant Cloud instance.
- **Rationale**: The `qdrant-client` provides a high-level API for managing collections and points. It simplifies the process of creating a collection and upserting vectors.
- **Alternatives considered**:
    - **Direct HTTP requests**: More complex and error-prone than using the official client library.

## 4. Python Script Structure

- **Decision**: The `main.py` script will be structured with the following functions as requested:
    - `get_all_urls(base_url)`: Takes a base URL and returns a list of all URLs to be scraped. This will likely involve recursively following links from the base URL.
    - `extract_text_from_url(url)`: Takes a URL, fetches the HTML, and returns the clean text.
    - `chunk_text(text, chunk_size, overlap)`: Takes text and returns a list of text chunks.
    - `embed(chunks)`: Takes a list of text chunks and returns a list of vector embeddings.
    - `create_collection(collection_name, vector_size)`: Creates a new collection in Qdrant.
    - `save_chunk_to_qdrant(collection_name, chunks, embeddings)`: Saves the chunks and their embeddings to the Qdrant collection.
    - `main()`: The main function that orchestrates the entire pipeline.
- **Rationale**: This structure is clear, modular, and easy to follow. Each function has a single responsibility.
- **Alternatives considered**:
    - **A single, monolithic function**: Would be much harder to read, test, and debug.
    - **A class-based approach**: Could be a good option, but for a single script, a functional approach is simpler.
