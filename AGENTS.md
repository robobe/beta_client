# Python guidelines

- Apply SOLID pragmatically: give code focused responsibilities and introduce abstractions only when they solve a real need.
- Use descriptive names and type hints for public interfaces. Keep functions small and behavior easy to test.
- Name meaningful protocol values, limits, and repeated configuration as constants. Obvious literals are fine.
- Validate input at boundaries and preserve useful errors. Avoid silently swallowing failures.
- Prefer the standard library and existing project patterns; avoid unnecessary dependencies and complexity.
- Test public behavior, including important invalid inputs and error paths.

# Mermaid diagrams

- In sequence-diagram message text, avoid bare semicolons (`;`); use a comma or `#59;` for a literal semicolon.
