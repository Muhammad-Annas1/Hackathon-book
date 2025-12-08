# Quickstart Guide: Building and Viewing the Book Locally

This guide provides instructions for setting up your environment to build and view the book locally. The book is built using Docusaurus.

## Prerequisites

- **Node.js**: Version 18 or higher (LTS recommended).
- **npm**: Comes with Node.js.
- **Git**: For cloning the repository.

## Setup

1.  **Clone the Repository**:
    ```bash
    git clone [repository-url]
    cd [repository-name]
    ```

2.  **Install Dependencies**:
    Navigate to the root of the repository and install the Docusaurus dependencies:
    ```bash
    npm install
    ```

## Running the Book Locally

To start the local development server and view the book in your browser:

1.  **Start Docusaurus**:
    ```bash
    npm start
    ```
    This will typically open a new tab in your web browser at `http://localhost:3000`. Any changes you make to the Markdown files will automatically reload the page.

## Building the Book for Production

To build the static HTML, CSS, and JavaScript files for deployment:

1.  **Build Docusaurus**:
    ```bash
    npm run build
    ```
    The generated static content will be located in the `build/` directory. This directory can then be deployed to a static hosting service like GitHub Pages.
