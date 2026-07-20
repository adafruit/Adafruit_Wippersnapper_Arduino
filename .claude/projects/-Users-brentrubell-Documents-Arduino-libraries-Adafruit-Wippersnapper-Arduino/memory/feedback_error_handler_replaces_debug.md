---
name: publishComponentError replaces WS_DEBUG_PRINTLN
description: When calling publishComponentError, remove the preceding WS_DEBUG_PRINTLN since publishComponentError already calls it internally
type: feedback
---

When adding `Ws.error_handler->publishComponentError()` calls, do NOT also include a `WS_DEBUG_PRINTLN` line before it — `publishComponentError` already prints the debug message internally.

**Why:** Avoids duplicate debug output; the error_handler already wraps the debug print.
**How to apply:** Any time you add a publishComponentError call to an error path, remove or skip the corresponding WS_DEBUG_PRINTLN.
