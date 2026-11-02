# codespell configuration

This directory holds the data files used by
[codespell](https://github.com/codespell-project/codespell) to spell-check the
Markdown documentation. The main configuration lives in
[`../.codespellrc`](../.codespellrc).

## How it runs in CI

A GitHub Actions workflow,
[`../.github/workflows/codespell.yml`](../.github/workflows/codespell.yml),
runs codespell on **Markdown files only** for every pull request and push to
`master` (and on demand from the *Actions* tab). If a misspelling is found the
job fails and prints the offending word, file, line, and suggested correction.
Fix the typo, or — for a false positive — add the word to `ignore-words.txt`
(see below), and push again.

## Running locally

From the repository root, run the same command CI uses:

```bash
pip install codespell
git ls-files '*.md' | xargs codespell -D - -D .codespell/dictionary.txt
```

`.codespellrc` (skip list and ignore-words) is loaded automatically. `-D -`
keeps codespell's built-in dictionary and the second `-D` appends the
project dictionary. Add `-w` to apply single-suggestion fixes automatically,
then review with `git diff`.

## Files

### `ignore-words.txt`

Correctly-spelled words that codespell would otherwise report as typos —
proper nouns, product names, and domain terms (for example `pincher`, from
"PhantomX Pincher Robot Arm").

- One lowercase word per line (matching is case-insensitive).
- Comment lines starting with `#` **are** allowed and are ignored by codespell.
- Add an entry here only when codespell flags a *correct* word (a false
  positive). For a genuine typo, fix the source text instead.

### `dictionary.txt`

Project-specific corrections that are checked in addition to codespell's
built-in dictionary (for example `gazbo->gazebo`). It is supplied on the command
line with `-D - -D .codespell/dictionary.txt`, where `-D -` keeps the built-in
dictionary and the second `-D` appends this file.

Format rules (these are strict — breaking them makes codespell error out):

- Every line **must** be a single `typo->correction` entry.
- **No comment lines and no blank lines** are allowed. codespell splits every
  line on `->`, so any line without exactly one `->` raises an error. This is
  why the explanation lives in this README instead of inside `dictionary.txt`.
- Use lowercase; codespell lower-cases both sides when loading the file.
- A correction may list multiple comma-separated suggestions
  (`typo->option1, option2`); with a single suggestion, `codespell -w` can apply
  the fix automatically.

#### How to add more words to `dictionary.txt`

> **Note:** `dictionary.txt` itself cannot contain this explanation, because a
> comment line has no `->` and makes codespell abort with
> `ValueError: not enough values to unpack`. That is why the guidance lives
> here in the README.

1. Open [`dictionary.txt`](dictionary.txt) and add one line per misspelling, in
   the form `typo->correction` (lowercase), for example:

   ```text
   recieve->receive
   teh->the
   ```

2. Keep it to genuine typos that codespell's built-in dictionary misses. If the
   word is actually correct (a product or proper name), add it to
   [`ignore-words.txt`](ignore-words.txt) instead.

3. Do **not** add comment or blank lines to `dictionary.txt`.

4. Verify the dictionary still loads and the docs are clean, from the
   repository root:

   ```bash
   git ls-files '*.md' | xargs codespell -D - -D .codespell/dictionary.txt
   ```

   A non-zero exit prints each offending word with its file, line, and
   suggested fix. Add `-w` to the command to apply single-suggestion fixes
   automatically, then review with `git diff`.
