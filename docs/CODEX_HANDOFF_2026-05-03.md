# Codex Handoff - 2026-05-03

## Session

- Date: 2026-05-03
- Branch: `codex/add-codex-handoff-doc-rule`
- Commit: pending
- Pull request: pending
- Related issue/logs: user requested persistent handoff docs for future Codex chats

## Summary

Added a repo-level requirement that future Codex sessions must create or update
a Markdown handoff file under `docs/` whenever they do meaningful work.

## Why It Matters

BlindNav work often happens across the Codex app, Pi terminal sessions, school
computer sessions, GitHub branches, and field logs. Raw Codex conversations may
not be easy to recover later, so the durable record should live in GitHub docs
and PRs.

## Files Or Logs Used

- `AGENTS.md`
- `docs/CODEX_HANDOFF_TEMPLATE.md`

## Evidence

The new `AGENTS.md` rule tells future Codex sessions to document meaningful
work:

```md
For any meaningful Codex work, create or update a Markdown handoff file under
`docs/` before finishing.
```

The template gives future sessions a consistent structure:

```md
## Summary
## Why It Matters
## Files Or Logs Used
## Evidence
## Validation
## Caveats
## Next Steps
```

## Validation

No runtime tests were needed because this is a documentation-only workflow
change.

## Caveats

- This rule depends on future Codex sessions reading `AGENTS.md`.
- It does not automatically sync raw local Codex conversations to Codex Cloud.
- Handoff docs should summarize sensitive logs instead of pasting secrets or
  full private raw logs.

## Next Steps

1. Push this branch.
2. Open a PR into `main`.
3. In future Codex work, include a handoff doc in the same PR unless explicitly
   told not to.
