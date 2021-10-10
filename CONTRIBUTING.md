# Contributing to xarm-commander

This document provides general information for contributing to xarm-commander.

## Format and Lint

The formatting style used in this project is Google with an indent of 4 (see `.clang-format`).
To format the source code, run the following:
```
clang-format -i -style=file *.cc
```

Currently, clang-tidy is used for linting (see `.clang-tidy`).
To lint the source code, run the following:
```
clang-tidy *.cc -- -Ilibs
```

## Commits

xarm-commander is maintained to be compatible with [Conventional Changelog](https://github.com/conventional-changelog/conventional-changelog), which structure Git commit messages in a way that allows automatic generation of changelogs.
Commit messages must be structured as follows:
```
<type>(<scope>): <subject>
<BLANK LINE>
<body>
<BLANK LINE>
<footer>
```

* `<type>`: A noun specifying the type of change, followed by a colon and a space. The types allowed are:
   * `feat`: A new feature
   * `fix`: A bug fix
   * `refactor`: Code change that neither fixes a bug or adds a feature (not relevant for end user)
   * `perf`: Changeimproves performance
   * `style`: Change does not affect the code (e.g., formatting, whitespaces)
   * `test`: Adding missing tests
   * `chore`: Change of build process or auxiliary tools
   * `docs`: Documentation only changes
* `<scope>`: Optional. A term of free choice specifying the place of the commit change, enclosed in parentheses. Examples:
   * `feat(binding-coap): ...`
   * `fix(cli): ...`
   * `docs: ...` (no scope, as it is optional)
* `<subject>`: A succinct description of the change, e.g., `add support for magic`
   * Use the imperative, present tense: "add", not "added" nor "adds"
   * Do not capitalize first letter: "add", not "Add"
   * No dot (.) at the end
* `<body>`: Optional. Can include the motivation for the change and contrast this with previous behavior.
   * Just as in the subject, use the imperative, present tense: "change" not "changed" nor "changes"
* `<footer>`: Optional. Can be used to automatically close GitHub Issues and to document breaking changes.
   * The prefix `BREAKING CHANGE: ` idicates API breakage (corresponding to a major version change) and everything after is a description what changed and what needs to be done to migrate
   * GitHub Issue controls such as `Fixes #123` or `Closes #4711` must come before a potential `BREAKING CHANGE: `.

Examples:
```
docs: improve how to contribute
```
```
feat(core): add support for magic

Closes #110
```
```
feat(core): add support for magic

Simplify the API by reducing the number of functions.

Closes #110
BREAKING CHANGE: Change all calls to the API to the new `do()` function.
```
