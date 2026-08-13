---
    description: Configuration for AI behavior when interacting with Codacy's MCP Server
    applyTo: '**'
---
---
# Codacy Rules
Configuration for AI behavior when interacting with Codacy's MCP Server

## Whenever a call to a Codacy tool that uses `repository` or `organization` as a parameter returns a 404 error
- Offer to run the `codacy_setup_repository` tool to add the repository to Codacy
- If the user accepts, run the `codacy_setup_repository` tool
- Do not ever try to run the `codacy_setup_repository` tool on your own
- After setup, immediately retry the action that failed (only retry once)
---