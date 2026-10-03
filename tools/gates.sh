#!/usr/bin/env bash
# gates.sh: the whole pre-commit gate in the dev container, detached from the calling shell: format, build, unit tests,
# script tests, quality. The output goes to log/tools/gate_all.log and the verdict to log/tools/gate_flag.txt ("X=0" is
# green); the script returns at once, so poll the flag. Do not edit the tracked tree while it runs.
cd "$(dirname "${BASH_SOURCE[0]}")/.."
mkdir -p log/tools; rm -f log/tools/gate_flag.txt
(setsid bash -c "./scripts/dev_shell.sh <<'EOS' > log/tools/gate_all.log 2>&1
make format && make build && make test && make test-scripts && make quality; echo X=\$? > /workspace/log/tools/gate_flag.txt
exit
EOS
" &)
