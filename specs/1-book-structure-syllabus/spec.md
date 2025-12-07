# Feature Specification: Book Structure & Syllabus Map

### Table of Contents
1.  📝 Feature Summary
2.  👥 User Scenarios & Testing
3.  ✅ Requirements
4.  🔑 Key Entities
5.  🏆 Success Criteria
6.  ❓ Open Questions/Decisions


**Feature Branch**: `1-book-structure-syllabus`
**Created**: 2025-12-06
**Status**: Draft

### 1. 📝 Feature Summary
This feature involves generating the foundational directory structure and file plan for a Docusaurus-based textbook on Physical AI & Humanoid Robotics. It includes creating the main `docs` directory, subdirectories for each book part, and a `sidebars.js` configuration to enable navigation based on the specified course modules. Additionally, it involves creating initial markdown files for each chapter with appropriate titles and placeholder content.

### 📂 Module Breakdown (from user input)
**Part 1: Foundations**
* **Chapter 1: The Embodied Intelligence Shift** (Intro to Physical AI, Digital vs. Physical laws, Sensors).
* **Chapter 2: The Hardware Stack** (Setting up the "Digital Twin" Workstation, Introduction to Jetson Orin & RealSense).

**Part 2: The Nervous System (ROS 2)**
* **Chapter 3: ROS 2 Architecture** (Nodes, Topics, Services).
* **Chapter 4: Bridging Python to Reality** (Writing `rclpy` controllers).
* **Chapter 5: URDF & Robot Description** (Defining the humanoid body).

**Part 3: The Digital Twin (Simulation)**
* **Chapter 6: Gazebo Physics** (Gravity, collision, world building).
* **Chapter 7: Unity for HRI** (Human-Robot Interaction visualization).

**Part 4: The AI-Robot Brain"
* **Chapter 8: AI Planning & Decision Making** (Integrating LLMs with ROS 2 for robot decision-making).
* **Chapter 9: Sensor Fusion & Perception** (Combining data from multiple sensors for environment understanding).
* **Chapter 10: Motion Planning & Control** (Path planning and execution for humanoid robots).

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Docusaurus Project Setup (Priority: P1)

As a user, I want the basic Docusaurus project structure to be created, including the main `docs` directory and subdirectories for each part of the book, so that I have a foundation for organizing the content.

**Why this priority**: This is a foundational step, necessary before any content can be added or structured within Docusaurus.

**Independent Test**: The existence of the `docs` directory and its `partX` subdirectories confirms the basic structure is in place.

**Dependencies**: None (this is a foundational setup).

**Acceptance Scenarios**:

*   **Given** no Docusaurus `docs` directory exists,
    **When** the setup process is run,
    **Then** the `docs` directory and `docs/part1`, `docs/part2`, `docs/part3`, `docs/part4` subdirectories are created.

---

### User Story 2 - Syllabus Navigation (Priority: P1)

As a user, I want a `sidebars.js` configuration that accurately reflects the book's module breakdown and chapters, so that the Docusaurus navigation is correctly generated and I can easily browse the content.

**Why this priority**: Correct navigation is crucial for the usability of the Docusaurus site and for readers to find chapters effectively.

**Independent Test**: The `sidebars.js` file exists and correctly lists the book parts and chapters.

**Dependencies**: User Story 1 (Docusaurus Project Setup) must be completed.

**Acceptance Scenarios**:

*   **Given** a Docusaurus project structure,
    **When** the `sidebars.js` is generated,
    **Then** it contains entries for 'Part 1: Foundations', 'Part 2: The Nervous System (ROS 2)', 'Part 3: The Digital Twin (Simulation)', 'Part 4: The AI-Robot Brain', and their respective chapters, correctly linked to the markdown files.

---

### User Story 3 - Chapter Content Creation (Priority: P2)

As a user, I want a markdown file created for each chapter with an initial title and placeholder content, so that I can begin writing the detailed material for each section of the book.

**Why this priority**: While structure and navigation are P1, the actual content creation can proceed once the foundation is laid.

**Independent Test**: Each specified chapter markdown file exists in the correct `docs/partX` directory and contains a chapter title.

**Dependencies**: User Story 1 (Docusaurus Project Setup) and User Story 2 (Syllabus Navigation) must be completed.

**Acceptance Scenarios**:

*   **Given** the Docusaurus project structure and `sidebars.js` are in place,
    **When** the chapter content tasks are executed,
    **Then** a markdown file is created for each chapter (e.g., `docs/part1/chapter1-embodied-intelligence-shift.md`, `docs/part2/chapter3-ros2-architecture.md`) with a suitable title and Frontmatter (id, title, sidebar_label, description).

---

## ❓ Open Questions/Decisions

- **Chapter File Overwriting**: When a chapter markdown file already exists, the system SHOULD overwrite the existing file to ensure consistency with the latest specification. This ensures that changes to the module breakdown are properly reflected in the generated content.
- **Invalid Module Breakdown Handling**: When the system encounters an invalid or malformed `Module Breakdown` input, it SHOULD fail fast with a clear error message identifying the specific issue, rather than attempting partial generation which could result in inconsistent or incomplete documentation structure.
## ✅ Requirements *(mandatory)*

### Functional Requirements
- **FR-001**: The system MUST create the top-level `docs` directory at the project root if it does not already exist.
- **FR-002**: The system MUST create subdirectories within `docs` for each major part of the book (e.g., `docs/part1`, `docs/part2`, `docs/part3`, `docs/part4`) if they do not already exist.
- **FR-003**: The system MUST generate a `sidebars.js` file at the Docusaurus project root that accurately defines the navigation structure based on the `Module Breakdown`.
- **FR-004**: The `sidebars.js` file MUST include each `Book Part` and its corresponding `Chapters` as navigable links, using the `id`, `title`, and `sidebar_label` from the chapter's Frontmatter.
- **FR-005**: The system MUST create a markdown file (`.md` or `.mdx`) for each chapter, with a consistent naming convention (e.g., `chapterX-chapter-title-slug.md`) within its respective `docs/partX` directory.
- **FR-006**: Each generated chapter markdown file MUST contain a markdown H1 title corresponding to its chapter title and the required Docusaurus Frontmatter (id, title, sidebar_label, description).
- **FR-007**: The system MUST ensure that all generated files (directories, `sidebars.js`, chapter markdown files) adhere to the formatting and content guidelines specified in `constitution.md`.

### Non-Functional Requirements
- **NFR-001 (Performance)**: The generation process for all files (directories, `sidebars.js`, 8 chapter markdown files) MUST complete within 60 seconds on a standard development machine.
- **NFR-002 (Reliability)**: The generation process MUST be idempotent; running it multiple times with the same input SHOULD produce the same output and not introduce errors or duplicate content.
- **NFR-003 (Maintainability)**: The generated `sidebars.js` and chapter markdown files MUST be easily maintainable and human-readable, adhering to standard Docusaurus practices.

### Architectural Constraints
- **AC-001**: The solution MUST integrate with Docusaurus as the primary documentation platform.
- **AC-002**: All content files MUST be Markdown or MDX compatible.
- **AC-003**: The navigation structure MUST be defined exclusively through `sidebars.js`.

### Key Entities & Attributes

- **Book Part**:
    - **Name**: (e.g., "Foundations", "The Nervous System (ROS 2)")
    - **Directory**: Corresponding subdirectory within `docs` (e.g., `docs/part1`)
    - **Chapters**: A collection of `Chapter` entities it contains.
- **Chapter**:
    - **Title**: The full title of the chapter (e.g., "The Embodied Intelligence Shift")
    - **ID**: A unique slug identifier for the chapter (e.g., `chapter1-embodied-intelligence-shift`)
    - **Sidebar Label**: A concise label for navigation (e.g., "Ch 1: Embodied AI")
    - **Description**: A brief summary of the chapter content for Frontmatter.
    - **File Path**: The expected markdown file path (e.g., `docs/part1/chapter1-embodied-intelligence-shift.md`)
- **Docusaurus Project Structure**: The filesystem layout required by Docusaurus for documentation, including the `docs` directory and `sidebars.js`.

## 🏆 Success Criteria & Validation *(mandatory)*

### Measurable Outcomes
- **SC-001**: All 4 required Docusaurus `partX` subdirectories (`docs/part1` to `docs/part4`) MUST be created.
- **SC-002**: The `sidebars.js` file MUST be generated and successfully configured to display all 4 `Book Parts` and their 8 specified `Chapters` in the Docusaurus navigation without errors (validated by `docusaurus build`).
- **SC-003**: Exactly 8 markdown files (one for each specified chapter) MUST be created, each located in the correct `docs/partX` subdirectory, with a valid markdown H1 title and complete Frontmatter (id, title, sidebar_label, description).
- **SC-004**: All generated files (`sidebars.js` and 8 chapter markdown files) MUST pass Markdown linting and Docusaurus build validation (exit code 0).