## 📜 Constitution for Physical AI & Humanoid Robotics Textbook

### Table of Contents
1.  🎯 Core Mission
2.  👥 Audience Profile
3.  ✅ Quality & Formatting Standards
4.  🤖 RAG & Personalization Hooks
5.  🚫 Non-Goals & Scope Exclusions
6.  🌐 System Architecture & API Design
7.  🔄 Review and Update Cycle


**Project:** AI-Native Textbook on Physical AI & Humanoid Robotics
**Target Platform:** Docusaurus (Markdown/MDX) deployed on GitHub Pages.
**Author:** Panaversity Hackathon Participant.

### 1. 🎯 Core Mission
Create a cutting-edge, academic yet practical textbook designed to bridge the gap between digital AI (LLMs/Agents) and the physical world (Robotics). The content must prepare students to control humanoid robots using ROS 2, Gazebo, and NVIDIA Isaac.

### 2. 👥 Audience Profile
* **Primary:** Computer Science students and developers transitioning into robotics.
*   **Knowledge Base:** Familiar with Python (intermediate), basic AI/ML concepts, and command-line interfaces.
*   **Prerequisites:** Readers should have a foundational understanding of programming logic and be comfortable with basic algebra. No prior experience with robotics hardware or advanced physics is assumed, as these will be introduced in the textbook.
* **Tone:** Technical, authoritative, yet accessible. Use "we" for instructional guidance.
* **Language:** American English.

### 3. ✅ Quality & Formatting Standards
*   **Markdown Only:** All output must be valid Markdown (`.md` or `.mdx`) for Docusaurus.
*   **Heading Structure**:
    *   `#` (H1): Reserved for the main chapter title. Each chapter file must have exactly one H1.
    *   `##` (H2): Major sections within a chapter.
    *   `###` (H3): Sub-sections.
    *   `####` (H4+): Use sparingly for fine-grained organization.
    *   Ensure logical hierarchy and sequential numbering where applicable (e.g., Chapter 1.1, 1.1.1).
*   **Interactive Elements:** Use Docusaurus admonitions frequently to highlight hardware constraints, important information, or warnings about potential mistakes. Examples:

    ```md
    :::tip
    This is a helpful tip!
    :::

    :::info
    Important information to note.
    :::

    :::warning
    Be careful with this step.
    :::
    ```
*   **Code Blocks:** All code must be Python (`rclpy`) or C++ (where strictly necessary) with clear, concise comments explaining non-obvious logic. Use ` ```python ` or ` ```bash ` syntax, and ensure code is runnable.
*   **Code Commenting**: Comments should explain *why* code does something, not *what* it does (unless the 'what' is complex). Avoid excessive or redundant comments.
*   **Internal and External Linking**: Use relative paths for internal links within the Docusaurus site. For external references, use standard Markdown link syntax `[Link Text](URL)` and ensure all URLs are valid and accessible.
* **Visuals:** Where a diagram is needed, insert a placeholder tag: `[Image: Diagram of X]`.
* **Equations:** Use LaTeX format (e.g., $F=ma$) for physics descriptions.

### 4. 🤖 RAG & Personalization Hooks
*   **Metadata:** Every chapter must start with Frontmatter. Example:
    ```yaml
    ---
    id: chapter-id
    title: Chapter Title
    sidebar_label: Short Label
    description: A brief summary of the chapter content.
    ---
    ```
* **Personalization Tags:** Structure content so it can later be filtered. Use comment markers like `` and `` for sections specific to physical hardware vs. simulation only.

### 5. 🚫 Non-Goals & Scope Exclusions
* **No Fluff:** Avoid generic AI hype. Focus on technical implementation (ROS 2 nodes, Isaac Sim setup).
*   **Hardware Agnostic (Mostly):** While focusing on Unitree/NVIDIA, explain concepts generally enough that they apply to other robots where possible.
*   **Avoid Future-Proofing:** Do not include content or features that are not explicitly required by the current project scope or specification. Prioritize clarity and directness over hypothetical future extensibility.

### 6. 🌐 System Architecture & API Design
* **Backend API Resilience:** The backend FastAPI server must handle external service failures gracefully. When Qdrant or other external services are unavailable, the system must provide fallback responses rather than throwing errors.
* **Error Handling:** All API endpoints should catch exceptions from external services (like Qdrant) and provide appropriate fallback behavior. For example, if the knowledge base is unavailable, the system should still respond to user queries with a graceful fallback message.
* **Frontend-Backend Communication:** The Docusaurus frontend communicates with the backend API at `http://127.0.0.1:8000` using standard HTTP requests. The chatbot component handles CORS and error states appropriately.
* **API Endpoints:** The backend provides `/query` and `/query-openai` endpoints that retrieve context from Qdrant when available, with fallback to general responses when the knowledge base is unavailable.

### 7. 🔄 Review and Update Cycle
*   **Regular Review:** The constitution should be reviewed quarterly or upon significant project shifts to ensure its continued relevance and accuracy.
*   **Amendment Process:** Any proposed amendments must be formally documented, discussed, and approved by project stakeholders (e.g., via an ADR process if significant).