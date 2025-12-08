# Data Model: Book Structure

This document outlines the content structure for the book. As this is a documentation project, the "data model" refers to the organization of the content itself, rather than a database schema.

## Key Entities

### Module
- **Description**: A top-level container for a major subject area in the book. Each module corresponds to a directory in the `docs/` folder (e.g., `docs/module-2/`).
- **Attributes**:
    - `title`: The name of the module (e.g., "Module 2: The Digital Twin (Gazebo & Unity)").
    - `directory`: The corresponding folder name.

### Chapter
- **Description**: A single instructional unit within a Module. Each chapter is a standalone Markdown file.
- **Attributes**:
    - `title`: The title of the chapter.
    - `filename`: The markdown file name (e.g., `chapter-1-gazebo-physics.md`).
    - `order`: The sequence of the chapter within the module.
- **Relationships**:
    - Belongs to one `Module`.

## Structure Example

```
docs/
├── module-1/
│   ├── chapter-1-core-concepts.md
│   └── ...
└── module-2/  <-- Module
    ├── chapter-1-gazebo-physics.md  <-- Chapter
    ├── chapter-2-unity-digital-twin.md
    └── chapter-3-sensor-simulation.md
```
