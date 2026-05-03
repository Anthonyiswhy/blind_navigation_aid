# Codex Handoff Template

Use this template whenever Codex does meaningful local work, opens a branch,
opens a PR, reviews field logs, or makes a repo-backed decision.

## Session

- Date:
- Branch:
- Commit:
- Pull request:
- Related issue/logs:

## Summary

Shortly describe what changed or what was diagnosed.

## Why It Matters

Explain how this affects BlindNav safety, latency, reliability, testing, field
workflow, or release readiness.

## Files Or Logs Used

- `path/to/file`
- `path/to/log`

## Evidence

Include small code snippets, log snippets, command output, or test results.
Do not include API keys, secrets, or full raw private logs.

```text
example output
```

## Validation

```bash
pytest tests/test_blindnav.py tests/test_blindnav_v326.py -q
```

Result:

```text
example: 174 passed
```

## Caveats

- What was not tested?
- What might be stale?
- What should the next Codex chat verify first?

## Next Steps

1. Next concrete action.
2. Follow-up validation.
3. GitHub branch or PR action if needed.
