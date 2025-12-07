<!--
---
Sync Impact Report
---
- **Version Change**: `none` -> `v1.0.0`
- **New Principles**:
  - Principle 1: High Technical Accuracy
  - Principle 2: Clarity for Target Audience
  - Principle 3: Verifiable and Consistent Content
  - Principle 4: Modular and Reproducible Content
- **Added Sections**:
  - Key Standards and Constraints
- **Removed Sections**:
  - Sections 2 and 3 from the template were replaced.
- **Templates Requiring Updates**:
  - `✅` .specify/templates/plan-template.md
  - `✅` .specify/templates/spec-template.md
  - `✅` .specify/templates/tasks-template.md
- **Follow-up TODOs**:
  - `[TODO(RATIFICATION_DATE)]`: Set the initial ratification date.
-->

# Book + RAG Chatbot on Physical AI & Humanoid Robotics Constitution

## Core Principles

### I. High Technical Accuracy
Content must be technically accurate, focusing on ROS 2, Gazebo, Unity, NVIDIA Isaac, and VLA. All explanations and examples should be precise and reflect current best practices.

### II. Clarity for Target Audience
The primary audience is Computer Science, AI, and robotics students. The material should be presented with clarity, assuming a foundational knowledge but not expert-level experience.

### III. Verifiable and Consistent Content
All content must be consistent with official documentation and accompanied by verifiable code examples. Claims must be source-verified, citing official docs or research papers in IEEE format. A minimum of 40% of sources should be from robotics/AI research or official documentation.

### IV. Modular and Reproducible Content
The book's content should be modular and structured for Docusaurus. All examples, especially the RAG chatbot, must be reproducible. The RAG implementation will use OpenAI Agents/ChatKit, FastAPI, Qdrant, and Neon.

## Key Standards and Constraints

- **Citation Style**: All claims must be source-verified using the IEEE citation style.
- **Source Requirements**: A minimum of 40% of all sources must be from robotics/AI research papers or official documentation.
- **Reproducibility**: All RAG examples must be fully reproducible using the specified stack: OpenAI Agents/ChatKit, FastAPI, Qdrant, and Neon.
- **Content Structure**: The book must be between 8 and 12 chapters (approximately 25,000–35,000 words) and formatted in Docusaurus-friendly Markdown.
- **Deployment**: The final Docusaurus site must be deployed and functional on GitHub Pages.
- **RAG Chatbot Functionality**: The chatbot must be able to answer questions using only the content from the book.
- **Code Accuracy**: All code for ROS 2, Gazebo, Isaac, and VLA pipelines must be verified and functional.
- **Generation Tooling**: Content generation will leverage Spec-Kit Plus and Claude Code.

## Governance

This Constitution is the single source of truth for project standards and principles. All contributions, reviews, and generated artifacts MUST adhere to it. Amendments require a documented proposal, review, and an update to the version number.

- **Compliance**: All PRs and reviews must verify compliance with these principles. Any deviation requires explicit justification and approval.
- **Success Criteria**: The project is considered successful when the Docusaurus build passes, the book is deployed, the RAG chatbot is functional and accurate, and all code is verified.

**Version**: v1.0.0 | **Ratified**: TODO(RATIFICATION_DATE) | **Last Amended**: 2025-12-07
