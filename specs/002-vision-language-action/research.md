# Research & Decisions for Docusaurus Architecture

This document records the research and decisions made for establishing the Docusaurus-based book architecture.

## 1. Decision: Docusaurus Folder Layout

### Decision
The project will adopt the standard Docusaurus v3 folder structure. All documentation content will reside within the `docs/` directory at the repository root. Each book module will be a subdirectory within `docs/`, and each chapter a markdown file within its respective module directory.

**Example Structure**:
```
docs/
├── module-1-ros2/
│   ├── chapter-1-core-concepts.md
│   └── ...
└── sidebar.js
```

### Rationale
This structure is the standard convention for Docusaurus, as confirmed by official documentation and community best practices. It is simple, logical, and well-understood. It leverages Docusaurus's content discovery and sidebar generation features effectively. Using subdirectories for modules provides a clean organization for a multi-chapter book.

### Alternatives Considered
- **Single directory**: Placing all markdown files in the root of `docs/`. Rejected because it would be disorganized for a book with multiple modules.
- **Monorepo approach**: Splitting each module into a separate Docusaurus instance. Rejected as overly complex for the current scale of the project. This might be considered in the future if the project grows significantly.

---

## 2. Decision: Code Block Formatting

### Decision
Code blocks will use Docusaurus's built-in syntax highlighting via Prism React Renderer. Specific language identifiers will be used for each block:
- **Python**: ` ```python `
- **URDF**: ` ```xml ` (as URDF is XML-based)
- **ROS Launch Files**: ` ```xml `
- **C++**: ` ```cpp `
- **Bash/Shell**: ` ```bash `

The `xml` language will be added to the `additionalLanguages` array in `docusaurus.config.js` to ensure it is highlighted correctly.

### Rationale
Docusaurus provides robust, out-of-the-box syntax highlighting that is easy to configure. Using the correct language identifier is crucial for readability and accuracy. The decision to use `xml` for URDF and launch files is based on their underlying file format. This approach requires minimal configuration while providing a high-quality reading experience.

### Alternatives Considered
- **Custom highlighters**: Using a different library for syntax highlighting. Rejected as it would add unnecessary complexity and maintenance overhead compared to the excellent built-in Prism support.

---

## 3. Decision: Technology Versions

### Decision
The project will standardize on the following major versions for its core technologies:
- **ROS 2**: Humble Hawksbill (LTS)
- **Docusaurus**: v3

### Rationale
- **ROS 2 Humble**: As a Long-Term Support (LTS) release, Humble provides stability, security updates, and maintenance until 2027. It is the standard for production-ready robotics applications and aligns with Ubuntu 22.04, a common development environment. Its focus on performance and stability makes it an ideal choice for teaching and for building reliable examples.
- **Docusaurus v3**: This is the latest major version, offering significant upgrades to core dependencies like React v18 and MDX v3. This ensures the project uses modern web technologies, benefits from performance improvements, and receives ongoing support and new features from the Docusaurus team. Docusaurus v2 is in maintenance mode, making v3 the forward-looking choice.

### Alternatives Considered
- **ROS 2 Rolling Ridley**: The "rolling" release of ROS 2. Rejected because it is a development release and lacks the long-term stability of an LTS version, which is not ideal for a book with examples that need to remain valid for a long time.
- **Docusaurus v2**: Rejected because it is the older version and will eventually stop receiving updates. Starting a new project on v2 would incur technical debt from day one.

---

## 4. Decision: Markdown Readability

### Decision
To ensure markdown files are readable and maintainable, the following practices will be adopted:
1.  **Keep Markdown Clean**: Avoid complex inline HTML or extensive inline JSX.
2.  **Use Components for Complexity**: For any complex, interactive, or reusable content, create a dedicated React component in `src/components` and import it into the MDX file.
3.  **Consistent Frontmatter**: Use YAML frontmatter consistently in every file to define metadata like `title` and `description`.
4.  **Global Styling**: Use the global `src/css/custom.css` for any custom styling needs rather than inline styles.

### Rationale
The goal is to keep the source markdown files as clean and semantic as possible. This makes it easier for authors to write and review content without getting bogged down in implementation details. Separating complex presentation into React components aligns with both Docusaurus and React best practices, improving modularity and maintainability.

### Alternatives Considered
- **Heavy use of inline JSX/HTML**: Allowing authors to embed complex logic directly in markdown. Rejected because it tightly couples content and presentation, making both harder to maintain and update.
