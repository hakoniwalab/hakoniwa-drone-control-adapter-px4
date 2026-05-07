# Usage Test Structure

## Purpose

This note explains how usage-oriented tests are structured across:

- `hakoniwa-drone-control-adapter`
- `hakoniwa-drone-control-adapter-px4`

The goal is not only to validate implementation, but also to keep executable
usage examples aligned with the public adapter contract.

## Design Rule

The usage example structure is split in two layers.

### 1. Adapter-side shared skeleton

Location:

- `hakoniwa-drone-control-adapter/test/`

Responsibility:

- express how the public adapter interfaces are intended to be used
- provide backend-agnostic usage flows
- avoid depending on PX4-specific types or PX4 config details

Examples:

- `usage_examples.cpp`
- `frequency_usage_examples.cpp`

### 2. Backend-side concrete runner

Location:

- `hakoniwa-drone-control-adapter-px4/test/`

Responsibility:

- load backend-specific config
- instantiate concrete backend implementations
- connect adapter-side shared usage skeletons to executable test binaries

Examples:

- `px4_usage_examples.cpp`
- `px4_frequency_usage_examples.cpp`

## Why This Split Exists

This split keeps two concerns separate.

- the public contract and its intended usage belong with the adapter
- the executable implementation details belong with the backend repository

That makes it possible to:

- preserve interface-level usage examples in the adapter repository
- execute those examples with PX4 today
- reuse the same shared usage skeletons for future backend implementations

## What These Tests Are

These tests are not strict numerical validation suites first.

They are primarily:

- executable usage examples
- contract-level integration checks
- backend-concretized samples

Therefore they intentionally focus on:

- call sequence
- mode usage
- scheduler/frequency usage
- finite output checks
- minimal semantic direction checks

and avoid overfitting to exact floating-point values where that would reduce
reusability across backend implementations.

## Current Coverage

The current usage-oriented test set covers:

- attitude-to-rate-to-torque flow
- altitude position mode
- altitude velocity mode
- horizontal velocity mode
- horizontal position mode
- multi-rate scheduler style execution using runtime frequencies

## Future Direction

When new backend implementations are added, the intended pattern is:

1. reuse the adapter-side shared usage skeleton when possible
2. add a backend-specific concrete runner
3. keep backend-specific fixture/config details outside the adapter repository
