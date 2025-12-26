---
id: 7
title: Debugging and Fixing RAG Retrieval
stage: refactor
date_iso: 2025-12-25
surface: agent
model: gemini-2.0-flash-exp
feature: 001-rag-chatbot-integration
branch: main
user: Annas
command: none
labels: ["rag", "qdrant", "metadata", "bugfix"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - backend/src/services/local_ingest.py
  - backend/src/services/qdrant_service.py
  - backend/src/services/llm_service.py
tests_yaml:
  - backend/test_rag_fix.py
  - backend/test_minimal_filter.py
  - backend/test_qdrant_logic.py
---

# Prompt
My ragchatbot is not giving answers of my book like give summarize the chapter 4 of module 1. ANalize it and make it working. chatbot should answer questions regarding the book

# Response
I've analyzed the RAG chatbot and identified why it's struggling with specific chapter summaries. The main issues were:
1. Retrieval limit too low (top_k=5).
2. Ingesting from live site instead of local files.
3. Lack of structural metadata (Module/Chapter).
4. Missing database indexes for metadata filtering.

I implemented the following:
- Created `local_ingest.py` to index local documentation with `module` and `chapter` metadata.
- Explicitly created Keyword indexes in Qdrant for these fields.
- Updated `QdrantService` to support structural filtering and deep retrieval.
- Updated `LLMService` to detect module/chapter patterns and retrieve the full context for summaries.

# Outcome
- Standardized metadata indexing for 59 documentation chunks.
- Verified successful retrieval and summarization of Module 1 Chapter 4 in test scripts.
- Resolved "400 Bad Request" filtering errors in Qdrant.
