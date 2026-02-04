# CLAUDE.md

## Rules

- Do NOT include Claude's signature (e.g. `Co-Authored-By: Claude ...`) in any generated code, commit messages, pull request descriptions, documentation, or any other output.
- After modifying any files, always run `pre-commit run --all-files` and apply the results before committing.
- When creating commits and pull requests, always use `tier4/bag_converter` (origin) as the base repository. Never push commits or create PRs against the upstream fork (`0x126/bag_converter`).
- Always write source code and documentation in English.
- Write PR descriptions that are comprehensive and detailed, but concise. Cover the problem, solution, and test plan without unnecessary verbosity.
