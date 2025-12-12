# Data Model for RAG Ingestion Pipeline

This document defines the data entities used in the RAG ingestion pipeline.

## 1. Content Chunk

Represents a segment of text extracted from a Docusaurus page.

- **Fields**:
    - `chunk_id` (string): A unique identifier for the chunk (e.g., a hash of the content).
    - `text` (string): The raw text content of the chunk.
    - `metadata` (dict): A dictionary containing metadata about the chunk.
        - `url` (string): The URL of the page from which the chunk was extracted.
        - `title` (string): The title of the page.
        - `chunk_index` (int): The index of the chunk within the page.

- **Validation Rules**:
    - `text` must not be empty.
    - `metadata.url` must be a valid URL.

## 2. Vector Record

Represents the stored version of a `Content Chunk` in the Qdrant vector database.

- **Fields**:
    - `id` (string): The `chunk_id` of the content chunk.
    - `vector` (array of floats): The vector embedding of the chunk's text.
    - `payload` (dict): The `metadata` from the `Content Chunk`.

- **Relationships**:
    - A `Vector Record` has a one-to-one relationship with a `Content Chunk`.
