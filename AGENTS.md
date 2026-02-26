# AGENTS.md

## Rules

- Do NOT include AI agent signatures (e.g. `Co-Authored-By: <agent name> ...`) in any generated code, commit messages, pull request descriptions, documentation, or any other output.
- After modifying any files, always run `pre-commit run --all-files` and apply the results before committing.
- When creating commits and pull requests, always use `tier4/bag_converter` (origin) as the base repository. Never push commits or create PRs against the upstream fork (`0x126/bag_converter`).
- Do not push directly to the main branch without permission. Always create a pull request first.
- Always write source code and documentation in English.
- Write PR descriptions that are comprehensive and detailed, but concise. Cover the problem, solution, and test plan without unnecessary verbosity.
- When writing code or documentation, be mindful of the GitHub Actions workflows configured in this repository and ensure changes do not cause them to fail.
- When investigating GitHub Actions workflow failures, always use the `gh` command (e.g. `gh run view`, `gh run view --log-failed`) to retrieve and read the actual workflow logs. Base your bug fixes on evidence from those logs, not on assumptions.
- Always obtain explicit developer approval before making any changes to the remote repository (e.g. pushing commits, creating/closing PRs or issues, commenting on PRs).
