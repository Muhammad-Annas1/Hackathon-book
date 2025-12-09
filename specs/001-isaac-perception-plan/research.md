# Research Findings: Module 3: The AI-Robot Brain (NVIDIA Isaac)

**Date**: 2025-12-09
**Feature**: [specs/001-isaac-perception-plan/spec.md](specs/001-isaac-perception-plan/spec.md)
**Plan**: [specs/001-isaac-perception-plan/plan.md](specs/001-isaac-perception-plan/plan.md)

This document consolidates key architectural and content-related decisions for Module 3, based on the feature specification and user-defined planning aspects.

## Decision: Docusaurus Folder Layout

*   **What was chosen**: A standard Docusaurus content structure, with a dedicated `docs/module-3-isaac/` directory for module-specific chapters. This structure will also include a `sidebar.js` configuration tailored for the module's navigation.
*   **Rationale**:
    *   Aligns with Docusaurus best practices for organizing multi-module content, promoting clarity and scalability.
    *   Provides clear separation of concerns, making content easy to locate, navigate, and manage for both authors and students.
    *   Simplifies future integration of other modules into the larger book structure.
*   **Alternatives considered**:
    *   **Single flat `docs/` structure**: Rejected due to potential for clutter and increased difficulty in managing content as the book grows.
    *   **Highly customized folder structures**: Rejected to maintain Docusaurus conventions, simplify maintenance, and ensure compatibility with Docusaurus tooling.

## Decision: Code Block Format

*   **What was chosen**: Standard Markdown code blocks with language highlighting (e.g., Python, C++, YAML). Emphasis will be placed on providing clear, concise, and runnable examples. For more complex, project-level examples, external code files will be referenced.
*   **Rationale**:
    *   Ensures high readability for students and simplifies the process of copying and pasting code examples.
    *   Leverages Docusaurus's built-in syntax highlighting capabilities for improved code presentation.
    *   Balances the need for self-contained examples with the practicality of managing larger codebases.
*   **Alternatives considered**:
    *   **Inline code**: Rejected for longer code snippets due to significantly reduced readability and maintainability.
    *   **Always external code files**: Rejected for simpler examples to keep the documentation self-contained and reduce context switching for the reader.

## Decision: Version Choices (ROS 2 Humble, Docusaurus v3)

*   **What was chosen**:
    *   **ROS 2 Distribution**: Humble Hawksbill (LTS).
    *   **Docusaurus Version**: v3.x (latest stable).
*   **Rationale**:
    *   **ROS 2 Humble**: As a Long-Term Support (LTS) release, Humble provides stability, extensive community support, and a predictable lifecycle, making it an ideal choice for an educational text.
    *   **Docusaurus v3**: Offers the latest features, performance improvements, and bug fixes for documentation site generation, ensuring a modern and efficient user experience.
    *   These versions align with current industry standards and provide a robust, forward-looking learning environment for students.
*   **Alternatives considered**:
    *   **Older ROS 2 distributions (e.g., Foxy, Galactic)**: Rejected due to approaching or past end-of-life status and potentially less relevance to current industry practices.
    *   **Docusaurus v2**: Rejected to leverage the latest features, security updates, and to future-proof the documentation project.

## Decision: Markdown Complexity vs. Readability

*   **What was chosen**: Prioritize readability and clarity over excessive Markdown complexity. Standard Markdown features will be used consistently, and Docusaurus-specific components will be employed judiciously, only when they significantly enhance the learning experience or presentation (e.g., for tabs, admonitions).
*   **Rationale**:
    *   Ensures the content remains accessible to students with varying levels of Markdown familiarity, reducing unnecessary cognitive load.
    *   Facilitates easier content creation and maintenance by keeping the formatting straightforward.
    *   Keeps the primary focus on the technical content itself, rather than deciphering complex or obscure formatting.
*   **Alternatives considered**:
    *   **Extensive use of highly customized Docusaurus components**: Rejected for general content to avoid over-engineering and to keep the content broadly portable.
    *   **Minimal Markdown (plain text)**: Rejected as it would compromise the structured, visually appealing, and engaging nature expected of a Docusaurus-based technical book.
