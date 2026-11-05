#!/usr/bin/env bash
# gates.sh [--docs]: the whole pre-commit gate in the dev container, detached from the calling shell: format, build,
# unit tests, script tests, quality. The output goes to log/tools/gate_all.log and the verdict to log/tools/gate_flag.txt
# ("X=0" is green); the script returns at once, so poll the flag. Do not edit the tracked tree while it runs.
# --docs: the gate for a change to Markdown files alone (the owner's rule of 2026-10-17): only the script tests, which
# read the documents (the tools a document cites, the scenarios against the Makefile); refused, with X=1, when anything
# but a *.md file differs from HEAD. The format, the build, the unit tests and the quality checks read no Markdown.
# --replay: the stack replayed on every kept bag under log/tools/replay/regression/<run>/ (tools/replay_stack.sh; the
# owner's decision of 2026-11-05, roadmap item 36: the estimators' poses within 0.05 m at p95 of the recorded
# flight's, K62), after the whole gate on a change to the estimators, the obstacle memory or the planner's input path;
# X=1 when a replay fails or no bag is kept. Some fifteen minutes for the two sets.
cd "$(dirname "${BASH_SOURCE[0]}")/.."
mkdir -p log/tools; rm -f log/tools/gate_flag.txt
if [[ "${1:-}" == "--replay" ]]; then
  (setsid bash -c '
    X=0; kept=(log/tools/replay/regression/*/)
    [[ -d "${kept[0]}" ]] || { echo "gates --replay: no kept bag under log/tools/replay/regression/"; X=1; }
    for dir in "${kept[@]}"; do [[ -d "$dir" ]] && { ./tools/replay_stack.sh "${dir%/}" || X=1; }; done
    echo X=$X > log/tools/gate_flag.txt' > log/tools/gate_all.log 2>&1 &)
  exit 0
fi
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
