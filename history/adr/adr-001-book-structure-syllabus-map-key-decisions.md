# ADR-001: Book Structure & Syllabus Map Key Decisions

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Proposed
- **Date:** 2025-12-06
- **Feature:** Book Structure & Syllabus Map
- **Context:** This ADR addresses critical architectural choices identified during the planning phase for the "Book Structure & Syllabus Map" feature, specifically concerning the Docusaurus-based textbook on Physical AI & Humanoid Robotics. These decisions impact the long-term maintainability, development velocity, and pedagogical effectiveness of the textbook content.

## Decision

This decision cluster covers three key areas:

1.  **ROS 2 Distribution**: We will use **ROS 2 Humble Hawksbill (LTS)** for all code examples and instructions.
2.  **Code Context for Examples**: We will primarily use **Proxy Robot (simplified, abstract examples)** for foundational concepts and introduce **Humanoid Robot (realistic, complex examples)** for advanced integration chapters.
3.  **Docusaurus Version**: We will use the **latest stable Docusaurus version** and commit to regular, managed updates.

## Consequences

### Positive

*   **ROS 2 Humble**: Ensures long-term stability, extensive community support, and a predictable environment for students, minimizing breakage due to API changes. This aligns with a textbook's need for a consistent learning experience.
*   **Proxy Robot Examples**: Reduces the initial learning curve for foundational ROS 2 and robotics concepts, allowing students to grasp core principles without being overwhelmed by hardware-specific complexities. Facilitates broader applicability beyond specific humanoid robot platforms.
*   **Latest Docusaurus Version**: Provides access to the newest features, performance optimizations, and security patches. This keeps the textbook platform modern and leverages the best available tooling for content delivery and interactive elements.

### Negative

*   **ROS 2 Humble**: May mean some examples cannot demonstrate the very latest features available in newer, non-LTS ROS 2 distributions. Content might require updates to reflect newer features if they become standard in future LTS releases.
*   **Proxy Robot Examples**: Advanced students might initially find the simplified examples less directly applicable to real-world humanoid robotics projects, potentially requiring additional effort to bridge the gap to complex systems. This will be mitigated by introducing humanoid-specific examples in later chapters.
*   **Latest Docusaurus Version**: Requires ongoing maintenance and potential adaptation of content or configurations with each major Docusaurus update, introducing a slight risk of breaking changes if not managed carefully.

## Alternatives Considered

*   **ROS 2 Distribution Alternatives**:
    *   **Iron Irwini (newer features)**: Rejected due to shorter support cycle and higher potential for breaking changes, which is not ideal for a stable textbook.
*   **Code Context for Examples Alternatives**:
    *   **Exclusively Humanoid Robot examples**: Rejected because it would significantly increase the initial learning barrier for beginners, potentially overwhelming them with complexity before core concepts are understood.
*   **Docusaurus Version Alternatives**:
    *   **Pin to an older LTS Docusaurus version**: Rejected as it would forgo new features, performance improvements, and security updates, potentially leading to a stale or less performant platform over time.

## References

- Feature Spec: D:\Programming\Q-4\Robotech-Book\specs\1-book-structure-syllabus\spec.md
- Implementation Plan: D:\Programming\Q-4\Robotech-Book\specs\1-book-structure-syllabus\plan.md
- Related ADRs: null
- Evaluator Evidence: null
