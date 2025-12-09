# Data Model for Book Content

This document defines the structure of the core content entities for the Docusaurus-based book.

## Entity: Module

Represents a top-level section of the book, equivalent to a "part" or "unit".

-   **Fields**:
    -   `title` (string, required): The user-facing name of the module.
    -   `directory_name` (string, required): The slugified name used for the folder (e.g., `module-1-ros2`).
-   **Relationships**:
    -   Has many `Chapter` entities.
-   **Validation Rules**:
    -   `directory_name` must be unique and follow the pattern `module-N-short-description`.

## Entity: Chapter

Represents a single content page within a `Module`. This corresponds to a single `.md` file.

-   **Fields**:
    -   `title` (string, required): The user-facing title of the chapter, defined in the markdown file's frontmatter.
    -   `description` (string, optional): A brief summary of the chapter's content, defined in the frontmatter.
    -   `file_name` (string, required): The name of the markdown file (e.g., `chapter-1-core-concepts.md`).
    -   `content` (string, required): The body of the chapter, written in Docusaurus-flavored Markdown (MDX).
-   **Relationships**:
    -   Belongs to one `Module`.
-   **Validation Rules**:
    -   Must begin with a YAML frontmatter block containing at least a `title`.
    -   `file_name` must be unique within its parent `Module`.

## Entity: Code Block

Represents a formatted block of source code embedded within a `Chapter`.

-   **Fields**:
    -   `language` (string, required): The language identifier for syntax highlighting (e.g., `python`, `xml`, `cpp`, `bash`).
    -   `code` (string, required): The raw code to be displayed.
    -   `title` (string, optional): An optional title for the code block, specified using the `title="..."` syntax in the opening fence.
-   **Relationships**:
    -   Embedded within one `Chapter`.
-   **Validation Rules**:
    -   `language` must be a valid language supported by the project's Prism configuration.
    -   Code must be well-formed for its specified language to ensure correct highlighting.
