# Research & Decisions for Humanoid Robotics Book

This document records the key architectural and content strategy decisions for the Docusaurus-based book.

## 1. Docusaurus Version and Theme

- **Decision**: Use Docusaurus v3 (latest stable version) with the default "classic" theme.
- **Rationale**: The classic theme is well-maintained, feature-rich, and provides a clean, modern documentation look-and-feel out of the box. It includes support for docs, a blog, pages, and all the features needed for the book. Sticking to the default reduces setup complexity and maintenance overhead.
- **Alternatives Considered**:
    - **Custom Theme**: Building a custom theme would be time-consuming and offers little pedagogical value for a book focused on robotics.
    - **Third-party themes**: Could introduce maintenance risks and potential compatibility issues with future Docusaurus versions.

## 2. Folder and File Naming Conventions

- **Decision**: A hierarchical and consistent naming convention will be used.
    - **Module Folders**: Each module will have a top-level folder under `docs/`. The folder name will be kebab-case (e.g., `docs/01-robotic-nervous-system`). The number prefix ensures correct ordering.
    - **Chapter Files**: Each chapter will be a Markdown file (`.md` or `.mdx`) within its module's folder. The file name will be prefixed with the chapter number (e.g., `01-introduction.md`, `02-ros-architecture.mdx`).
    - **Assets**: All static assets (images, diagrams, videos) will be stored in a global `static/assets` directory, with subfolders for each module (e.g., `static/assets/module-1/`).
- **Rationale**: This structure is intuitive, scalable, and maps directly to the book's content outline. It also works seamlessly with Docusaurus's sidebar generation from folder structure.

## 3. Integration of Media (Code, Diagrams, Images)

- **Decision**: Standard Docusaurus/MDX features will be used.
    - **Code Snippets**: Use standard Markdown fenced code blocks with language identifiers for syntax highlighting. For runnable examples, consider integrating a tool like `react-live` if interactivity is needed, but default to static snippets for simplicity.
    - **Diagrams**: Use Mermaid JS for flowcharts, sequence diagrams, and architectural diagrams. Mermaid is supported by Docusaurus and can be written directly in Markdown.
    - **Images/GIFs**: Use standard Markdown image syntax, referencing assets from the `static/assets` directory.
- **Rationale**: Relying on built-in Docusaurus and MDX capabilities ensures the widest compatibility and easiest authoring experience.

## 4. Navigation Structure

- **Decision**:
    - **Sidebar**: The primary navigation will be a docs sidebar, automatically generated from the folder structure. Each module will be a collapsible category.
    - **Navbar**: The top navbar will link to the "Introduction", "Modules" (the docs root), and a "GitHub" repository link.
    - **Search**: Docusaurus's built-in Algolia search will be configured and enabled.
- **Rationale**: This is a standard and effective navigation pattern for documentation websites, providing both structured browsing via the sidebar and powerful search capabilities.

## 5. Content Organization

- **Decision**:
    - **`docs/`**: All book content (modules and chapters) will reside here.
    - **`src/`**: Any custom React components (e.g., for interactive diagrams or examples) will be in `src/components`.
    - **`static/`**: All static assets like images, URDF files for download, or datasets will be here.
    - **Code Examples**: Small, illustrative code snippets will be embedded directly in the Markdown files. Larger, complete ROS 2 packages or projects for students to download will be stored in a separate `examples/` directory at the root of the repository and linked from the text.
- **Rationale**: This separation of content, components, and static assets is a standard Docusaurus best practice that keeps the project organized and maintainable.
