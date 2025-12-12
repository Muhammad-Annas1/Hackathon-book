# Tasks for RAG Ingestion Pipeline

**Branch**: `001-rag-ingestion-pipeline` | **Date**: 2025-12-12 | **Plan**: [plan.md](./plan.md)
**Spec**: [spec.md](./spec.md) | **Research**: [research.md](./research.md) | **Data Model**: [data-model.md](./data-model.md)

## Summary

This document outlines the tasks required to implement the RAG Ingestion Pipeline. Tasks are organized by phases, primarily following user story priorities (P1, P2, P3) from the feature specification.

## Implementation Strategy

The implementation will follow an MVP-first approach, focusing on delivering User Story 1 (Content Extraction) as the initial functional increment. Subsequent user stories (Embedding Generation, Storing Embeddings) will be delivered incrementally.

## Dependency Graph (User Story Completion Order)

1.  User Story 1 (P1): Content Extraction from Docusaurus
2.  User Story 2 (P2): Embedding Generation
3.  User Story 3 (P3): Storing Embeddings in Qdrant

## Parallel Execution Opportunities

- Tasks within the "Setup" phase can be partially parallelized.
- Tasks within each User Story phase that operate on independent files or logic can be parallelized.

---

## Phase 1: Setup - Project Initialization

**Goal**: Prepare the project environment and initial file structure.

- [ ] T001 Create `backend` directory in the repository root.
- [ ] T002 Create `tests` directory in the repository root.
- [ ] T003 Create `requirements.txt` in the `backend` directory with initial dependencies: `uv`, `cohere`, `qdrant-client`, `beautifulsoup4`, `requests`.
- [ ] T004 Create `main.py` in the `backend` directory.
- [ ] T005 Create `test_main.py` in the `tests` directory.

---

## Phase 2: Foundational - Environment and Basic Structure

**Goal**: Set up the Python environment and basic script structure.

- [ ] T006 Initialize Python project with `uv` in the `backend` directory, if necessary.
- [ ] T007 Install project dependencies using `uv pip install -r backend/requirements.txt`.
- [ ] T008 Implement basic `.env` loading in `backend/main.py` for API keys and Qdrant URL.

---

## Phase 3: User Story 1 - Content Extraction from Docusaurus (P1)

**Goal**: Extract all textual content from a Docusaurus website.

**Independent Test Criteria**: The `get_all_urls` and `extract_text_from_url` functions can correctly identify and extract text content from a given Docusaurus URL.

- [ ] T009 [P] [US1] Implement `get_all_urls(base_url)` function in `backend/main.py` to recursively find all relevant URLs within the Docusaurus site.
- [ ] T010 [P] [US1] Implement `extract_text_from_url(url)` function in `backend/main.py` to fetch HTML and extract clean text using `requests` and `BeautifulSoup`.
- [ ] T011 [US1] Integrate `get_all_urls` and `extract_text_from_url` into the `main` function for initial content extraction.
- [ ] T012 [US1] Write unit tests for `get_all_urls` in `tests/test_main.py` to verify correct URL discovery.
- [ ] T013 [US1] Write unit tests for `extract_text_from_url` in `tests/test_main.py` to verify correct text extraction and cleaning.

---

## Phase 4: User Story 2 - Embedding Generation (P2)

**Goal**: Convert extracted content into vector embeddings.

**Independent Test Criteria**: The `chunk_text` and `embed` functions can correctly process text and generate valid Cohere embeddings.

- [ ] T014 [P] [US2] Implement `chunk_text(text, chunk_size, overlap)` function in `backend/main.py` based on research findings for optimal chunking.
- [ ] T015 [P] [US2] Implement `embed(chunks)` function in `backend/main.py` to generate Cohere embeddings for a list of text chunks.
- [ ] T016 [US2] Integrate `chunk_text` and `embed` into the `main` function after content extraction.
- [ ] T017 [US2] Write unit tests for `chunk_text` in `tests/test_main.py` to verify correct text chunking.
- [ ] T018 [US2] Write integration tests for `embed` in `tests/test_main.py` to verify Cohere API interaction and embedding generation.

---

## Phase 5: User Story 3 - Storing Embeddings in Qdrant (P3)

**Goal**: Store generated embeddings and metadata in Qdrant.

**Independent Test Criteria**: The `create_collection` and `save_chunk_to_qdrant` functions can successfully interact with Qdrant to store vector records.

- [ ] T019 [P] [US3] Implement `create_collection(collection_name, vector_size)` function in `backend/main.py` to create a Qdrant collection.
- [ ] T020 [P] [US3] Implement `save_chunk_to_qdrant(collection_name, chunks, embeddings)` function in `backend/main.py` to upsert vectors and metadata into Qdrant.
- [ ] T021 [US3] Integrate `create_collection` and `save_chunk_to_qdrant` into the `main` function after embedding generation.
- [ ] T022 [US3] Write unit tests for `create_collection` in `tests/test_main.py` to verify Qdrant collection creation logic.
- [ ] T023 [US3] Write integration tests for `save_chunk_to_qdrant` in `tests/test_main.py` to verify Qdrant interaction and data storage.

---

## Phase 6: Final Integration and Polish

**Goal**: Complete the end-to-end pipeline, add error handling, and ensure robustness.

- [ ] T024 Finalize `main` function orchestration for the entire RAG ingestion pipeline.
- [ ] T025 Add comprehensive error handling and logging throughout `backend/main.py`.
- [ ] T026 Update `quickstart.md` with any additional setup or execution details.
- [ ] T027 Run end-to-end integration test of the entire pipeline.
- [ ] T028 Ensure compliance with all success criteria (SC-001 to SC-005).
