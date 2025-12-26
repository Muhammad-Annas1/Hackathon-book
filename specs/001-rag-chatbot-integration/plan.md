# Implementation Plan: RAG Chatbot Integration

**Branch**: `001-rag-chatbot-integration` | **Date**: 2025-12-15 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `specs/001-rag-chatbot-integration/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

This plan outlines the creation of a RAG chatbot backend using FastAPI. The backend will serve the existing Docusaurus frontend, providing answers to user questions based on book content. As per the feature specification, the implementation will use OpenRouter for LLM responses, Gemini for embeddings, and Qdrant for vector retrieval. This represents a deviation from the stack mentioned in the project constitution (which specifies OpenAI and Neon), and this deviation is noted and justified below.

## Technical Context

**Language/Version**: Python 3.11
**Primary Dependencies**: FastAPI, Qdrant-client, httpx (for OpenRouter), google-generativeai, uv
**Storage**: Qdrant Cloud (Vector DB)
**Testing**: Pytest
**Target Platform**: Generic Linux server (for backend deployment)
**Project Type**: Web application (backend component)
**Performance Goals**: < 5s p95 response time for a standard query.
**Constraints**: Must use the user-specified stack. API keys and credentials must not be hardcoded and should be managed via environment variables.
**Scale/Scope**: Single chatbot for one book, designed for a moderate user load.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **I. High Technical Accuracy**: PASS. The chosen stack is capable of meeting the technical requirements.
- **II. Clarity for Target Audience**: PASS. The plan is clear and aimed at a technical implementer.
- **III. Verifiable and Consistent Content**: PASS. The plan adheres to creating verifiable outcomes.
- **IV. Modular and Reproducible Content**: **VIOLATION**. The constitution specifies the RAG implementation will use "OpenAI Agents/ChatKit, FastAPI, Qdrant, and Neon". This plan uses OpenRouter and Gemini embeddings instead of OpenAI, and Qdrant instead of Neon, per the feature specification. This is a justified deviation as the `/sp.specify` command provides overriding requirements for this feature.

## Project Structure

### Documentation (this feature)

```text
specs/001-rag-chatbot-integration/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Web application structure
backend/
├── .env.example
├── .uv/
├── pyproject.toml
├── src/
│   ├── main.py
│   ├── core/
│   │   └── settings.py   # For loading .env
│   ├── services/
│   │   ├── qdrant_service.py
│   │   └── llm_service.py
│   └── api/
│       └── v1/
│           └── chat.py
└── tests/
    ├── integration/
    └── unit/

# The existing Docusaurus frontend remains at the root
docs/
src/
...
```

**Structure Decision**: The project already has a frontend (Docusaurus) and this feature adds a distinct Python backend. Therefore, a `backend/` directory will be created to house the FastAPI application, keeping it separate from the existing frontend codebase, as per the user's request in the `/sp.plan` arguments.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| RAG stack deviation (OpenRouter/Gemini vs. OpenAI/Neon) | The explicit feature specification (`/sp.specify` command) for this work package mandates the use of OpenRouter and Gemini models. | The constitution's stack is a valid alternative, but the direct user instruction for this specific feature takes precedence over the general guideline. |
