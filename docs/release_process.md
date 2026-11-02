# How A Release Is Cut

A release is a tag `vMAJOR.MINOR.PATCH` on `main` with a `CHANGELOG.md`
entry and a GitHub release page; environment asset bundles carry their own
`environment-assets-*` tags and are not part of it. This page is the whole
procedure, in order, with every place the version is written, so that
nothing is forgotten: v0.6.0 shipped with `package.xml` still at 0.5.0 and
v0.7.0 had to say so.

The release itself — the tag, the push, the release page, any recording or
post — is the owner's decision and the owner's word starts it. An agent
prepares everything up to the release commit on that word and stops there
unless the word covers the tag and the push too
([`docs/autonomous_development.md`](autonomous_development.md)).

## 1. Decide What Ships

- The version. MINOR for a release that adds capability (every release so
  far), PATCH for a repair of a shipped release with nothing new, MAJOR when
  the owner says so.
- The one-line title, honest about what was reached: v0.7.0 said "VOXL 2
  readiness by documentation", not "support", because no board had run the
  stack. The title is the tag message and the release page's title.
- What is in it: the roadmap items closed since the last release (their
  closing lines carry the commit and the runs), the specification entries
  added or changed (the change log at the end of
  [`docs/specification.md`](specification.md)), the register rows that
  moved ([`docs/technical_debt.md`](technical_debt.md)), and the known
  limitations the entry must state.

## 2. The Code Is Final Before The Series

Everything that changes a flight goes in before the acceptance flights.
After them only the release records (step 4) change, so the flights are
flights of the released code. Check before flying:

- `main`, the tree clean, the last gate green (`log/tools/gate_flag.txt`
  reads `X=0`), `grep -c "this commit" docs/specification.md` reads 0.
- The host is quiet: nothing of weight besides the flights (v0.7.0 lost nine
  attempts of a night to the real-time factor with no visible foreign
  process; `tools/fly_until_valid.sh` reads the host before each attempt).

## 3. The Acceptance Series

The acceptance configuration of [`docs/specification.md`](specification.md)
(R1, R2, R3, F21): the clean Urban location, without GNSS, under the
imperfections profile, the repository's defaults. Five single urban
point-to-point flights on the stereo set and five on the lidar set, one at a
time through `tools/fly_until_valid.sh` (three attempts a counted flight;
an attempt the real-time factor or the watchdog voids is flown again under
the next number, and is listed in the entry with its cause):

```bash
./scripts/stop_sim.sh
./tools/fly_until_valid.sh <next run number> 3            # stereo, five times
./tools/fly_until_valid.sh --lidar <next run number> 3    # lidar, five times
```

Each counted flight is read by its verdict lines (`log/tools/run_rNNNN.log`):
no crash, the true position at the goal's acknowledgement within the 2 m
capture radius, the mean speed against R3's target (within the 20 percent
allowance; a series under the target is a row of the register's attempts
table). A `FAIL` in a counted flight is a finding, never a reason to fly
again: it is analysed and, if the code changes, the whole series is flown
again on the changed code. The series table, the scripts that flew it and
the verdict lines go into `log/tools/release_vXYZ/` with a `.keep` file at
its top (the weekly pruning deletes anything loose): v0.7.0's is
`log/tools/release_v070/series.txt`.

## 4. The Release Commit

One commit, on the code the series flew, titled `release vX.Y.Z: <title>`.
Every place the version lives:

| Where | What changes |
|---|---|
| `drone_city_nav/package.xml` | `<version>X.Y.Z</version>` (missed in v0.6.0) |
| `CHANGELOG.md` | a new `## vX.Y.Z (YYYY-MM-DD)` section above the previous release, under `## Unreleased` emptied of what ships: what is in it and why it matters, the laws in force, the series (the code's commit, the runs, the mean speeds against the targets, the true position at the goal, the voided attempts with their causes, the path of the series table), the known limitations, the compatible `environment-assets-*` tags, the package version |
| `README.md`, "Releases" | "The current release is `vX.Y.Z`: …" rewritten for this release |
| `docs/roadmap.md` | every item closed since the last release: its pending wording ("Not yet in a release (after vA.B.C)" or a bare "closed on …") becomes "Shipped in [vX.Y.Z](https://github.com/formiat/px4-ros2-drone-nav/releases/tag/vX.Y.Z) on YYYY-MM-DD; closed on …" |
| `docs/technical_debt.md` | rows that name "the next release" or the release's series |
| `log/tools/release_vXYZ/` | the series table and scripts, with `.keep` |

Then `tools/release_check.sh X.Y.Z` reads every one of them and the tag's
absence, and prints a line per check; the commit waits for every line to
read `OK`. The gate is the full one (`./tools/gates.sh`, `package.xml` is
not Markdown), `X=0` in `log/tools/gate_flag.txt`; then the commit, with
explicit paths.

## 5. Tag, Push, Publish (The Owner's Word)

```bash
git tag -a vX.Y.Z -m "vX.Y.Z: <title>"
git push origin main
git push origin vX.Y.Z
gh release create vX.Y.Z --title "vX.Y.Z: <title>" --notes-file <(sed -n '/^## vX.Y.Z /,/^## v/p' CHANGELOG.md | sed '1d;$d')
```

The release page's text is the changelog entry, nothing else. No build
artifacts are attached: the code builds from source and the assets are
released separately. After the page exists, `tools/release_check.sh X.Y.Z
--published` reads the tag, the page and that the page's text begins with
the entry.

## 6. After

- The memory and the journal of the session record the release (the
  commit, the tag, the page, the series).
- A recording or a post is a separate decision of the owner, never part of
  the release: recordings are of the stereo set only and run one to one
  with simulation time (A11); a post for LinkedIn and other places without
  Markdown is written one paragraph a line with no hard wraps; r/robotics is
  not a venue for simulation posts (the places that are: r/ROS, ROS
  Discourse, the PX4 forum, LinkedIn).
- Work after the release goes under `## Unreleased` in `CHANGELOG.md` as it
  lands, and a closed roadmap item reads "Not yet in a release (after
  vX.Y.Z)" until the next one ships, so step 4 finds it by that phrase.

## What Went Wrong Before

| Release | What was missed | The check that catches it now |
|---|---|---|
| v0.6.0 | `package.xml` stayed at 0.5.0 | `release_check.sh`: the package version |
| v0.7.0 | nine stereo attempts voided by the real-time factor at night, the cause never found | the host read before the series (step 2); the attempts listed in the entry |
| v0.5.0, v0.6.0 | the roadmap's "Not yet in a release" lines found one by one by hand | `release_check.sh`: no such line may remain |
