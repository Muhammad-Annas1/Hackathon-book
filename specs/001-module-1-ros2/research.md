# Research Notes: Module 1: The Robotic Nervous System (ROS 2)

## Decisions

### Docusaurus for Book Architecture
- **Decision**: Use Docusaurus for the book architecture.
- **Rationale**: Docusaurus is a popular and easy-to-use tool for building documentation websites. It provides excellent support for Markdown-based content, versioning, search functionality, and is highly customizable. Its capabilities align well with the project's goal of creating a structured and easily navigable book.
- **Alternatives Considered**:
    - **GitBook**: Considered for its focus on book-like structures but offers less flexibility in customization and integration compared to Docusaurus.
    - **MkDocs**: A simpler alternative for static site generation, but Docusaurus offers more out-of-the-box features tailored for documentation.

### Docusaurus Folder Layout
- **Decision**: The Docusaurus folder layout will follow the standard Docusaurus v3 structure.
- **Rationale**: Adhering to the standard structure simplifies development, maintenance, and collaboration. It leverages Docusaurus's conventions for organizing documentation, pages, and static assets, making it easier for new contributors to understand and extend the project.

### Code Block Format
- **Decision**: Code blocks will be formatted using standard Markdown syntax with language identifiers (e.g., ```python).
- **Rationale**: This approach ensures broad compatibility, readability, and proper syntax highlighting across various Markdown renderers and within Docusaurus itself. It aligns with best practices for technical documentation.

### Version Choices (ROS 2 Humble, Docusaurus v3)
- **Decision**: ROS 2 Humble and Docusaurus v3 will be used.
- **Rationale**: Both are current, stable, and widely supported versions of their respective technologies. Using recent versions ensures access to the latest features, performance improvements, and security updates, while also providing a relevant learning experience for the target audience.

### Markdown Complexity vs. Readability
- **Decision**: Markdown complexity will be kept to a minimum to ensure readability and maintainability.
- **Rationale**: Prioritizing readability ensures that the content is accessible to students and reduces the overhead for content creators. Complex Markdown features that do not significantly enhance clarity or functionality will be avoided to maintain a clean and consistent presentation.
