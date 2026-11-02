#!/bin/bash
# shellcheck disable=SC2317  # the predicates below run through expect()
# release_check.sh X.Y.Z [--published]: every place a release's version lives, read before the release commit
# (docs/release_process.md step 4): the package version, the changelog section, the readme's current release, no
# roadmap item left "Not yet in a release", no specification row left unstamped, the evidence folder with its .keep,
# the tree clean on main, the tag absent; with --published, the tag on HEAD and on origin and the GitHub release
# page present with the changelog entry's text. Prints one line a check, OK or FAIL, and exits non-zero on any FAIL.
set -u
cd "$(dirname "$0")/.." || exit 1
version=${1:-}
[[ "$version" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]] || { echo "usage: $0 X.Y.Z [--published]" >&2; exit 2; }
published=0; [[ "${2:-}" == "--published" ]] && published=1
tag="v$version"; folder="log/tools/release_v${version//./}"; fail=0
expect() {  # expect DESCRIPTION COMMAND...: one line, OK when the command succeeds
  local description=$1; shift
  if "$@"; then echo "OK   $description"; else echo "FAIL $description"; fail=1; fi
}
no_line() { ! grep -q "$1" "$2"; }
evidence_kept() { [ -d "$folder" ] && [ -f "$folder/.keep" ]; }
on_main() { [ "$(git rev-parse --abbrev-ref HEAD)" = "main" ]; }
tree_clean() { [ -z "$(git status --short)" ]; }
tag_exists() { git rev-parse -q --verify "refs/tags/$tag" > /dev/null; }
tag_absent() { ! tag_exists; }
tag_on_head() { [ "$(git rev-parse "$tag^{commit}" 2>/dev/null)" = "$(git rev-parse HEAD)" ]; }
tag_on_origin() { git ls-remote --tags origin "refs/tags/$tag" 2>/dev/null | grep -q "$tag"; }
page_exists() { gh release view "$tag" --json body -q .body > /dev/null 2>&1; }
page_carries_entry() {
  local body first
  body=$(gh release view "$tag" --json body -q .body 2>/dev/null) || return 1
  first=$(sed -n "/^## $tag (/,/^## v/p" CHANGELOG.md | sed '1d;$d' | grep -m1 -v '^$')
  [ -n "$first" ] && [ "${body#*"$first"}" != "$body" ]
}
expect "package.xml carries <version>$version</version>" grep -q "<version>$version</version>" drone_city_nav/package.xml
expect "CHANGELOG.md has a section '## $tag (date)'" grep -q "^## $tag (" CHANGELOG.md
expect "README.md names $tag as the current release" grep -q "The current release is \`$tag\`" README.md
expect "docs/roadmap.md has no 'Not yet in a release' line left" no_line "Not yet in a release" docs/roadmap.md
expect "docs/specification.md has no row left stamped 'this commit'" no_line "this commit" docs/specification.md
expect "$folder/ exists with .keep" evidence_kept
expect "on main" on_main
if [ "$published" -eq 0 ]; then
  expect "the tree is clean" tree_clean
  expect "the tag $tag does not exist yet" tag_absent
else
  expect "the tag $tag exists" tag_exists
  expect "the tag $tag is on HEAD" tag_on_head
  expect "the tag $tag is on origin" tag_on_origin
  expect "the GitHub release $tag exists" page_exists
  expect "the release page's text carries the changelog entry's first line" page_carries_entry
fi
exit $fail
