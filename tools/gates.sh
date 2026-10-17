#!/usr/bin/env bash
# gates.sh [--docs]: the whole pre-commit gate in the dev container, detached from the calling shell: format, build,
# unit tests, script tests, quality. The output goes to log/tools/gate_all.log and the verdict to log/tools/gate_flag.txt
# ("X=0" is green); the script returns at once, so poll the flag. Do not edit the tracked tree while it runs.
# --docs: the gate for a change to Markdown files alone (the owner's rule of 2026-10-17): only the script tests, which
# read the documents (the tools a document cites, the scenarios against the Makefile); refused, with X=1, when anything
# but a *.md file differs from HEAD. The format, the build, the unit tests and the quality checks read no Markdown.
cd "$(dirname "${BASH_SOURCE[0]}")/.."
mkdir -p log/tools; rm -f log/tools/gate_flag.txt
if [[ "${1:-}" == "--docs" ]]; then
  other=$( { git diff --name-only HEAD; git ls-files --others --exclude-standard; } | grep -v '\.md$' || true)
  if [[ -n "$other" ]]; then
    printf 'gates --docs refused: not only Markdown differs from HEAD:\n%s\n' "$other" > log/tools/gate_all.log
    echo X=1 > log/tools/gate_flag.txt
    exit 0
  fi
  (setsid bash -c "./scripts/dev_shell.sh <<'EOS' > log/tools/gate_all.log 2>&1
make test-scripts; echo X=\$? > /workspace/log/tools/gate_flag.txt
exit
EOS
" &)
  exit 0
fi
(setsid bash -c "./scripts/dev_shell.sh <<'EOS' > log/tools/gate_all.log 2>&1
make format && make build && make test && make test-scripts && make quality; echo X=\$? > /workspace/log/tools/gate_flag.txt
exit
EOS
" &)
