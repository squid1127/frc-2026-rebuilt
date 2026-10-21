---
description: Guidelines for making changes to the codebase, including adding, removing, and modifying code
# applyTo: 'Guidelines for making changes to the codebase, including adding, removing, and modifying code' # when provided, instructions will automatically be added to the request context when the pattern matches an attached file
---

<!-- Tip: Use /create-instructions in chat to generate content with agent assistance -->

- Keep code well typed using appropriate type annotations, avoiding Any and implicit return and param types where it makes sense
- Always use relative imports when importing other parts of the codebase
- Use from or import to import packages depending on which is optimal
- Provide logging in modules in the codebase using existing patterns, mainly where logging is useful
- Avoid verbose and per-tick logging, as it can overwhelm the logger

Validation:
- Use `ruff check` (*). Running `ruff check --fix and --unsafe-fixes` is also helpful
- Check the language server for errors
- Run `robotpy test` (*)

(*) Try running in venv, first in `.venv`, then default out to global python `py -m`, or running directly, and stick with the method that works