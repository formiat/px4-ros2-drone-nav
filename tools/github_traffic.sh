#!/usr/bin/env bash
# github_traffic.sh: the repository's traffic on GitHub kept, one JSON line a call in log/tools/traffic/traffic.jsonl
# (roadmap item 62): the stars, forks and watchers, the views and clones of the last 14 days by day, the referrers and
# the most read paths. GitHub holds 14 days and no more, so the weekly review of the forks runs this; a step taken
# to be found (a list's row, the documentation site) is then read by the referrers it brings. Needs `gh` signed in
# with push access to the repository of the checkout's origin. Host; read-only on GitHub.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
out=log/tools/traffic; mkdir -p "$out"; touch "$out/.keep"
api() { gh api "repos/{owner}/{repo}$1"; }
jq -c -n --arg at "$(date -u +%FT%TZ)" \
  --argjson repository "$(api "" | jq '{stars: .stargazers_count, forks: .forks_count, watchers: .subscribers_count}')" \
  --argjson views "$(api /traffic/views)" --argjson clones "$(api /traffic/clones)" \
  --argjson referrers "$(api /traffic/popular/referrers)" --argjson paths "$(api /traffic/popular/paths)" \
  '{at: $at, repository: $repository, views: $views, clones: $clones, referrers: $referrers, paths: $paths}' \
  >> "$out/traffic.jsonl"
tail -n 1 "$out/traffic.jsonl" | jq -r '"\(.at): stars \(.repository.stars), forks \(.repository.forks); 14 days: \(.views.count) views by \(.views.uniques), \(.clones.count) clones by \(.clones.uniques); referrers: \([.referrers[] | "\(.referrer) \(.uniques)"] | join(", "))"'
