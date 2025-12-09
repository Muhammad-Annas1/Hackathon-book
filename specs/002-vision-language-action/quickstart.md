# Quickstart: Contributing a New Chapter

This guide provides a quick overview for authors to contribute a new chapter to the book, ensuring it aligns with the established standards.

## 1. Prerequisites

-   You have cloned the repository.
-   You are on the `main` branch and have pulled the latest changes.
-   You have Node.js and Yarn installed.

## 2. Create a Feature Branch

All new content should be developed on a feature branch. Use the naming convention `[initials]/[short-description]`, for example: `aa/add-ros-actions-chapter`.

```bash
git checkout -b aa/add-ros-actions-chapter
```

## 3. Create the Chapter File

1.  Navigate to the appropriate module directory inside `docs/`. For example, if you are adding a chapter to `module-1-ros2`, go to `docs/module-1-ros2`.
2.  Create a new markdown file. Use the naming convention `chapter-N-short-description.md`.
3.  Add the standard frontmatter to the top of your file. At a minimum, you must include a `title`.

    ```markdown
    ---
    title: My New Chapter Title
    description: A brief and informative description of this chapter.
    ---

    Your content starts here...
    ```

## 4. Write Your Content

-   Follow the writing style and tone of the existing chapters.
-   Use standard Markdown for content.
-   For code blocks, use the correct language identifiers as defined in the project's `research.md`.

    ```markdown
    ```python
    # This is a Python code block
    print("Hello, ROS 2!")
    ```

    ```xml
    <!-- This is a URDF or Launch file code block -->
    <robot name="my_robot"></robot>
    ```

## 5. Update the Sidebar

To make your new chapter appear in the navigation, you must add it to the `docs/sidebar.js` file. Find the correct module section and add an entry for your new file. The ID is typically the file path relative to the `docs/` directory without the extension.

**Example `docs/sidebar.js` modification**:
```javascript
// ...
    'Module 1: ROS 2': [
      'module-1-ros2/chapter-1-core-concepts',
      'module-1-ros2/my-new-chapter-file-name', // Add your new chapter here
    ],
// ...
```

## 6. Preview Your Changes

Run the Docusaurus development server to preview your chapter and ensure it renders correctly.

```bash
yarn install
yarn start
```

Open your browser to `http://localhost:3000` to see your changes.

## 7. Submit a Pull Request

Once you are satisfied with your new chapter, commit your changes, push your branch, and open a Pull Request to the `main` branch for review.
