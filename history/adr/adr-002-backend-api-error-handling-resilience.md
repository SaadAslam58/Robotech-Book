# ADR-002: Backend API Error Handling & Resilience Pattern

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-26
- **Feature:** Backend API Resilience
- **Context:** This ADR addresses critical architectural choices for the "Robotech Book RAG API" backend, specifically concerning error handling when external services (Qdrant vector database) are unavailable. These decisions impact the long-term reliability, user experience, and maintainability of the textbook's AI assistant functionality.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

This decision implements a resilient error handling pattern for the backend API:

1. **Graceful Degradation**: When Qdrant vector database is unavailable, the API provides fallback responses instead of throwing errors.
2. **Try-Catch Wrapper Pattern**: All Qdrant calls are wrapped in try-catch blocks with appropriate fallback behavior.
3. **Fallback Context**: When knowledge base retrieval fails, the system continues to function with a general AI response rather than failing completely.
4. **Error Logging**: All retrieval errors are logged for debugging while maintaining user experience.

## Consequences

### Positive

*   **Enhanced User Experience**: Users no longer see error messages when the knowledge base is unavailable, maintaining a smooth interaction with the chatbot.
*   **System Reliability**: The API remains functional even when external dependencies fail, providing a more robust user experience.
*   **Maintainability**: Clear error handling patterns make the codebase easier to debug and extend.
*   **Graceful Degradation**: The system degrades gracefully to a general AI assistant when the specific knowledge base is unavailable, rather than becoming completely non-functional.

### Negative

*   **Reduced Context**: When Qdrant is unavailable, responses are not based on the specific textbook content, potentially reducing their relevance.
*   **Additional Complexity**: The try-catch patterns add slight complexity to the code, though this is offset by improved reliability.
*   **Monitoring Requirements**: Need to monitor error logs to detect when Qdrant is unavailable to address underlying issues.

## Alternatives Considered

*   **Alternative A: Hard Fail Pattern**: API would return 500 errors when Qdrant is unavailable. Rejected because it provides a poor user experience with error messages instead of helpful responses.
*   **Alternative B: Silent Fail Pattern**: API would return empty responses when Qdrant is unavailable. Rejected because it would appear as if the system is not working, rather than gracefully degrading.
*   **Alternative C: Cache-First Pattern**: Implement a local cache as primary data source. Rejected as it adds significant complexity for a prototype system and would require additional infrastructure.

## References

- Feature Spec: null
- Implementation Plan: null
- Related ADRs: ADR-001 (Book Structure & Syllabus Map Key Decisions)
- Evaluator Evidence: null