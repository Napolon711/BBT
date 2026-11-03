# Domain Docs

How the engineering skills should consume this repo's domain documentation.

## Before exploring, read these

- `GLOSSARY.md` at the repo root.
- `docs/adr/` for ADRs that touch the area being explored.

If these files do not exist, proceed silently. The domain-modeling skill creates them lazily when terms or decisions are resolved.

## File structure

This is a single-context repo:

```
/
├── GLOSSARY.md
├── docs/adr/
└── src/
```

## Use the glossary's vocabulary

When naming a domain concept in an issue title, refactor proposal, or test name, use the term defined in `GLOSSARY.md`. If the concept is not defined, note the gap for domain modeling.

## Flag ADR conflicts

If an output contradicts an existing ADR, surface the conflict explicitly rather than silently overriding it.
