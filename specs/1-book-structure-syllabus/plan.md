# Implementation Plan: Book Structure & Syllabus Map

### Table of Contents
1.  ⚙️ Plan Context & Constraints
2.  PROJECT EXECUTION PLAN
    *   Architecture Sketch: Content Life Cycle
    *   Section Structure: Chapter to Execution Phase Mapping
    *   Research Approach
    *   Quality Validation
3.  Decisions Needing Documentation
4.  Testing Strategy
    *   Structural Test for `sidebars.js`
    *   Content Integrity Test
    *   RAG & Personalization Tags Integrity Test
    *   Code Example Verification
    *   Docusaurus Build Test
5.  🚀 Technical Details


**Feature Branch**: `1-book-structure-syllabus`
**Created**: 2025-12-06
**Status**: Draft
**Governing Document**: D:\Programming\Q-4\Robotech-Book\.specify\memory\constitution.md
**Specification**: D:\Programming\Q-4\Robotech-Book\specs\1-book-structure-syllabus\spec.md

---

### 1. ⚙️ Plan Context & Constraints

*   **Governing Document:** **Constitution for Physical AI & Humanoid Robotics Textbook** (`.specify/memory/constitution.md`). All standards for tone, Markdown, code formatting, visuals, and RAG/Personalization hooks are fully enforced as per Section 3: Quality & Formatting Standards and Section 4: RAG & Personalization Hooks in the Constitution.
*   **Target Content:** Chapters as defined in the **Specification: Book Structure & Syllabus Map** (`specs/1-book-structure-syllabus/spec.md`). Specifically, the `Module Breakdown` section and `User Stories` (US1, US2, US3) in `spec.md`.

---

### 2. PROJECT EXECUTION PLAN

**Create:**

*   **Architecture Sketch: Content Life Cycle (Sense -> Plan -> Act)**
    *   **Sense**: This phase involves gathering raw information and conducting thorough research. Sources will include academic papers on robotics and AI, official documentation for ROS 2, Gazebo, NVIDIA Isaac Sim, and relevant open-source codebases. The focus will be on evidence tracing to ensure accuracy and academic rigor as per the Constitution.
    *   **Plan**: In this phase, the gathered research will be structured into logical chapters and sections. This includes mapping the content to the Docusaurus project structure, defining appropriate Docusaurus admonitions for interactive elements, and planning for the integration of RAG and Personalization Tags.
    *   **Act**: The final phase involves generating the actual Markdown/MDX content for each chapter. This includes embedding Python (`rclpy`) and C++ code blocks with clear comments, ensuring LaTeX formatting for equations, and placing image placeholder tags where diagrams are needed. All content will adhere strictly to the Constitutional guidelines for formatting, tone, and technical accuracy.

*   **Section Structure: Chapter to Execution Phase Mapping**
    *   **Research Phase**: Initial source gathering and evidence tracing for all chapters.
    *   **Foundation Phase (Part 1)**:
        *   Chapter 1: The Embodied Intelligence Shift
        *   Chapter 2: The Hardware Stack
    *   **Analysis Phase (Parts 2 & 3)**:
        *   Chapter 3: ROS 2 Architecture
        *   Chapter 4: Bridging Python to Reality
        *   Chapter 5: URDF & Robot Description
        *   Chapter 6: Gazebo Physics
        *   Chapter 7: Unity for HRI
    *   **Synthesis Phase (Part 4)**:
        *   Part 4: The AI-Robot Brain (placeholder chapters, for integration and capstone content)

*   **Research Approach**
    *   **Source Prioritization**: Prioritize official documentation (ROS 2, NVIDIA, Docusaurus), peer-reviewed academic publications, and reputable open-source project repositories.
    *   **Tools**: Utilize `WebSearch` for broad topic exploration and `WebFetch` for in-depth content extraction from specific URLs.
    *   **Rigor**: Cross-reference information from multiple authoritative sources to validate technical details and best practices (e.g., `rclpy` syntax, Omniverse USD commands). Ensure proper citation practices (internal referencing, not explicit external citations in final output unless specified by Constitution).
    *   **Ethical Sourcing**: All information will be sourced ethically and used in a manner consistent with academic integrity.

*   **Quality Validation**
    *   **Markdown Linting**: Automated checks to ensure all `.md` and `.mdx` files adhere to CommonMark and Docusaurus-specific Markdown standards.
    *   **Code Snippet Verification**:
        *   Syntax Highlighting: Confirm ` ```python ` or ` ```bash ` are correctly applied.
        *   Linting: Basic linting for Python and C++ code examples.
        *   Executability (where feasible): Simple tests for isolated code snippets to ensure they are functional (e.g., `rclpy` examples).
    *   **LaTeX Equation Verification**: Validation of LaTeX syntax for mathematical expressions (`$F=ma$`).
    *   **RAG & Personalization Tags Presence**: Automated checks to ensure every chapter has required Frontmatter (`id`, `title`, `sidebar_label`, `description`) and that conditional personalization tags (`<!-- simulation-only -->` and `<!-- hardware-only -->`) are correctly used and balanced.
    *   **Content Consistency**: Review for adherence to the "Technical, authoritative, yet accessible" tone and American English language as per the Constitution.
    *   **Technical Accuracy**: Manual or automated cross-referencing of technical facts with researched sources.

**Decisions needing documentation:**

*   **ROS 2 Distribution**:
    *   **Options**: Humble Hawksbill (LTS, stable) vs. Iron Irwini (newer features, shorter support cycle).
    *   **Trade-offs**: Humble offers long-term stability and extensive community support, ideal for a textbook. Iron provides access to the latest features but may introduce more breaking changes and require more frequent updates to content.
            *   **Rationale**: We will use **ROS 2 Humble Hawksbill (LTS)**. This ensures long-term stability, extensive community support, and a predictable environment for students, minimizing breakage due to API changes. This aligns with a textbook's need for a consistent learning experience. While it may mean some examples cannot demonstrate the very latest features available in newer, non-LTS ROS 2 distributions, the stability is paramount for a textbook.
*   **Code Context for Examples**:
    *   **Options**: Proxy Robot (simplified, abstract examples focusing on core concepts) vs. Humanoid Robot (realistic, complex examples demonstrating full integration with a specific robot model).
    *   **Trade-offs**: Proxy Robot is easier for beginners to grasp initial concepts but may lack the real-world applicability. Humanoid Robot examples offer greater realism and direct utility for advanced students but could increase the learning curve for foundational topics.
            *   **Rationale**: We will primarily use **Proxy Robot (simplified, abstract examples)** for foundational concepts, reducing the initial learning curve. **Humanoid Robot (realistic, complex examples)** will be introduced for advanced integration chapters to provide real-world applicability. This mitigates the risk of overwhelming beginners while still providing advanced content.
*   **Docusaurus Version**:
    *   **Options**: Use the latest stable Docusaurus version vs. pin to a specific older LTS version.
    *   **Trade-offs**: Latest version provides new features and bug fixes but might introduce breaking changes. An older LTS ensures stability but might miss out on improvements.
            *   **Rationale**: We will use the **latest stable Docusaurus version** and commit to regular, managed updates. This provides access to the newest features, performance optimizations, and security patches, keeping the textbook platform modern. While it requires ongoing maintenance, the benefits of updated tooling outweigh the risks of being locked into an older version.

**Testing strategy:**

*   **Structural Test for `sidebars.js`**:
    *   Verify `sidebars.js` exists at the root of the Docusaurus project.
    *   Automated parsing to confirm all specified `Book Parts` and `Chapters` from the `Specification` are correctly listed as `items` or `labels`.
    *   Check for correct hierarchical nesting and unique `id` values.
    *   Validate that all linked markdown files (e.g., `docs/part1/chapter1-embodied-intelligence-shift.md`) exist at their specified paths.
    *   Docusaurus `docusaurus start` (or `build`) should run without navigation-related warnings/errors.
*   **Content Integrity Test**:
    *   **File Existence**: Verify all `.md` files for chapters are created in their respective `docs/partX` directories.
    *   **Frontmatter Validation**: Check each `.md` file for the presence and correct format of `id`, `title`, `sidebar_label`, and `description` Frontmatter.
    *   **Title Presence**: Ensure each `.md` file contains at least one `#` (H1) or `##` (H2) markdown title.
    *   **Placeholder Check**: Scan for any residual placeholder text (e.g., `[Image: Diagram of X]`, `[NEEDS CLARIFICATION]`).
*   **RAG & Personalization Tags Integrity Test**:
    *   Automated regex or AST parsing to ensure `<!-- simulation-only -->` and `<!-- hardware-only -->` tags are present where appropriate and are correctly opened and closed.
    *   Verify that these tags do not break Markdown rendering.
*   **Code Example Verification**:
    *   Automated syntax checking for code blocks (e.g., `python -m py_compile` for Python snippets).
    *   Manual review of selected code snippets for clarity, correctness, and adherence to best practices for `rclpy` or C++.
*   **Docusaurus Build Test**:
    *   Execute `npm run build` within the Docusaurus project root.
    *   Ensure the build completes successfully with exit code 0.
    *   Review build output for any warnings related to broken links, unrendered Markdown, or configuration issues.

---

### 3. 🚀 TECHNICAL DETAILS

**Phased Execution:** The drafting must be organized sequentially into the following phases:
1.  **Research:** Initial source gathering and evidence tracing.
2.  **Foundation:** Writing introductory and setup chapters (Part 1).
3.  **Analysis:** Detailed implementation chapters (Parts 2, 3, 4).
4.  **Synthesis:** Integration and Capstone chapters (Part 4), finalizing references.
