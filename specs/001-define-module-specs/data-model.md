# Data Model (Content Structure)

This document defines the content structure for the "Physical AI & Humanoid Robotics" book, based on the Docusaurus framework.

## Key Entities

- **Module**: A top-level learning category. It is represented as a directory in the `docs/` folder and a category in the Docusaurus sidebar.
- **Chapter**: A specific lesson within a module. It is represented as a Markdown file (`.md` or `.mdx`) inside a module directory.
- **Asset**: A static file associated with the book, such as an image, diagram, or downloadable code example.

## Content Hierarchy (Folder Structure)

The repository will follow a structure that is optimized for Docusaurus's content discovery and sidebar generation.

```
humanoid-robotics-book/
├── docs/
│   ├── 01-robotic-nervous-system/
│   │   ├── 01-intro-physical-ai.md
│   │   ├── 02-ros-architecture.mdx
│   │   ├── 03-installing-ros.md
│   │   ├── 04-building-packages.md
│   │   ├── 05-humanoid-communication.md
│   │   ├── 06-urdf-definition.md
│   │   ├── 07-launch-files.md
│   │   └── 08-bridging-llms.mdx
│   ├── 02-digital-twin/
│   │   ├── 01-simulation-theory.md
│   │   │   ... (and so on for all chapters)
│   ├── 03-ai-robot-brain/
│   │   ├── ...
│   └── 04-vision-language-action/
│       └── ...
├── src/
│   ├── components/
│   │   └── # Custom React components for interactive elements
│   └── css/
│       └── custom.css
├── static/
│   └── assets/
│       ├── module-1/
│       │   └── # Images and assets for Module 1
│       └── ...
├── docusaurus.config.js
└── package.json
```

## Entity: Module

- **Description**: A collection of related chapters forming a cohesive learning unit.
- **Attributes**:
    - `title`: The full title of the module (e.g., "The Robotic Nervous System (ROS 2)").
    - `order`: An integer prefix on the folder name (e.g., `01-`) to control sidebar order.
- **Representation**: A directory under `docs/`.

## Entity: Chapter

- **Description**: A single page of content within a module.
- **Attributes**:
    - `title`: The full title of the chapter (e.g., "Introduction to Physical AI & Embodied Intelligence").
    - `order`: An integer prefix on the file name (e.g., `01-`) to control order within a module.
    - `content`: Markdown/MDX content, including text, code snippets, images, and diagrams.
- **Representation**: A `.md` or `.mdx` file. Using `.mdx` allows for the embedding of custom React components.

## Entity: Asset

- **Description**: Any non-text file used in the book.
- **Attributes**:
    - `type`: The file type (e.g., PNG, GIF, URDF, ZIP).
    - `path`: The location of the file within the `static/assets/` directory.
- **Representation**: A file stored in the `static/` directory and referenced via a relative path in Markdown/MDX.
