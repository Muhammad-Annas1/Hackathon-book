# Tasks: RAG Chatbot Integration

**Input**: Design documents from `specs/001-rag-chatbot-integration/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2)
- Include exact file paths in descriptions

## Phase 1: Project Setup

**Purpose**: Project initialization and basic structure for the backend service.

- [ ] T001 Create the backend project directory: `backend/`
- [ ] T002 Create a `pyproject.toml` file in `backend/` with initial dependencies: `fastapi`, `uv`, `python-dotenv`, `pydantic`.
- [ ] T003 Initialize the `uv` virtual environment in the `backend/` directory by running `uv venv`.
- [ ] T004 Create the core application directory structure: `backend/src/api/v1`, `backend/src/core`, `backend/src/services`.
- [ ] T005 Create an example environment file at `backend/.env.example` listing `OPENROUTER_API_KEY`, `GEMINI_API_KEY`, `QDRANT_URL`, and `QDRANT_API_KEY`.

---

## Phase 2: Foundational Backend Services

**Purpose**: Core services that MUST be complete before ANY user story can be implemented.

**⚠️ CRITICAL**: No user story work can begin until this phase is complete.

- [ ] T006 Implement settings management in `backend/src/core/settings.py` to load variables from the `.env` file using Pydantic.
- [ ] T007 [P] Implement the embedding service in `backend/src/services/embedding_service.py` with a function to generate Gemini embeddings for a given text.
- [ ] T008 [P] Implement the retrieval service in `backend/src/services/qdrant_service.py` with a function to query the top-k context chunks from Qdrant for a given vector.
- [ ] T009 [P] Implement the LLM service in `backend/src/services/llm_service.py` to build a prompt and get a response from OpenRouter.

**Checkpoint**: Foundation ready - user story implementation can now begin.

---

## Phase 3: User Story 1 - Get Answers from Full Book Content (Priority: P1) 🎯 MVP

**Goal**: A user can ask a question in the chat UI and get a relevant answer synthesized from the entire book's content.

**Independent Test**: The `/api/v1/chat` endpoint can be called with a question; it returns a valid answer based on content retrieved from Qdrant.

### Implementation for User Story 1

- [ ] T010 [US1] Define the Pydantic models for the chat request and response in `backend/src/api/v1/schemas.py`, based on `data-model.md`.
- [ ] T011 [US1] Implement the `/api/v1/chat` endpoint in `backend/src/api/v1/chat.py`. This will orchestrate the calls to the embedding, Qdrant, and LLM services.
- [ ] T012 [US1] Create the main application file `backend/src/main.py`, import the chat router, and configure CORS middleware to allow frontend requests.

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently via an API client.

---

## Phase 4: User Story 2 - Get Answers from Selected Text (Priority: P2)

**Goal**: A user can highlight text and ask a question to get an answer based only on that selection.

**Independent Test**: The `/api/v1/chat` endpoint can be called with a question *and* a `context` string; it returns an answer based only on the provided context.

### Implementation for User Story 2

- [ ] T013 [US2] Modify the chat endpoint in `backend/src/api/v1/chat.py` to accept an optional `context` field in the request body.
- [ ] T014 [US2] Add logic to the endpoint: if `context` is provided, bypass the Qdrant retrieval step and use the `context` string directly to build the LLM prompt.

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently.

---

## Phase 5: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories.

- [ ] T015 Add robust error handling to the `/api/v1/chat` endpoint for scenarios like external API failures or invalid input.
- [ ] T016 [P] Add structured logging to key functions in the services and API endpoint.
- [ ] T017 [P] Create a `README.md` file in the `backend/` directory with detailed setup and usage instructions.
- [ ] T018 Run `quickstart.md` validation to ensure the setup process is correct.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: Must be completed first.
- **Foundational (Phase 2)**: Depends on Setup. Blocks all user stories.
- **User Stories (Phases 3-4)**: Depend on Foundational.
- **Polish (Phase 5)**: Depends on all desired user stories being complete.

### User Story Dependencies

- **User Story 1 (P1)**: Depends on Phase 2.
- **User Story 2 (P2)**: Depends on User Story 1, as it modifies the same endpoint.

### Within Each Phase

- In Phase 2, tasks T007, T008, and T009 can be developed in parallel after T006 is complete.
- In other phases, tasks should be completed sequentially unless marked with [P].

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1.  Complete Phase 1: Setup.
2.  Complete Phase 2: Foundational.
3.  Complete Phase 3: User Story 1.
4.  **STOP and VALIDATE**: Test the `/api/v1/chat` endpoint with questions requiring book-wide context.
5.  The backend is now ready for frontend integration for the MVP.

### Incremental Delivery

1.  Deliver MVP (US1).
2.  Add User Story 2 functionality.
3.  Add Polish and final documentation.
Each stage results in a testable, more feature-rich backend.
