# Autonomous Development And Debugging: The Rules For Agents

How this project is developed by agents with the least possible call on
people, written down from the way it has actually been done since September
2026: long unattended sessions, hours of flights, dozens of commits, with the
owner reading reports and setting direction. Every rule here was learned on
this repository; the dated lessons at the end say where. The rules are the
owner's where they say so; the rest is practice an agent follows unless the
owner says otherwise.

The contract in one paragraph: the owner states what and why, in a few
lines; the agent turns it into a self-contained brief (the goal), verifies
every fact it rests on, decides every open question itself and records the
decision, builds the smallest change that satisfies the brief, proves it by
measurement (tests, gates, flights), writes the numbers into the registers
in the same commit, and reports honestly, deviations and doubts first. Nothing
in the work waits for a person; anything that must be the owner's (a release,
a purchase, a direction) is outside the goal and named as such.

## 1. The Two Requirements, And Measurement First

- The project has two requirements and no third (the owner, 2026-09-19): the
  vehicle never crashes (any contact with a surface is a crash, a landing on
  the ground apart) and always reaches the goal, judged by the truth of the
  simulator, not by its own estimate. Everything else the mission check
  measures is diagnostics: it is read, it never decides.
- The one figure the programme targets is the mean flight speed, and its
  value is the measured one ("the expected speed is the real one", 2026-10-24;
  the target mean of 2026-10-30, specification R3). A number nobody measured
  is not written anywhere: not a threshold, not a magnitude, not a tolerance.
  A parity bound is the measured spread, written in the register, never
  loosened to pass. A magnitude without a local source is closed by record,
  not invented ("never from a guess", roadmap item 35).
- A shortfall is a finding about the stack, never a reason to soften the
  test, the profile or the threshold. Parameters of the flight are never
  changed to pass an acceptance or to make a video.

## 2. The Goal: The Brief An Agent Works From

The agent's working memory does not survive a long session: the context is
compacted every few hours. The goal file is what the agent has when it wakes
up, so it is written to be read cold (the `write-goal` method; the files of
2026-10-24, 2026-10-29 and 2026-10-30 are the examples, under the owner's
`goals_done/`).

- One root file with the common rules, the order of the parts, the recording
  and the stop section; one file a part with its own facts, decisions,
  stages and stop section. A part starts only when the previous one is done
  or closed by record by its own criteria.
- Every fact is verified when the goal is written: `path:line`, commit
  hashes, line counts, tool availability, the baselines with the runs they
  came from. Nothing is written from memory of an earlier conversation.
- Every open question is closed at writing time by a decision with a
  one-line reason, numbered (Р1, Р2 ...) so the journal can refer to it, or
  by an instruction how to decide during the work (what to measure, what
  counts). Never "ask the owner".
- Every stage has Why, How (commands, paths, parameters, what to record) and
  Success (numbers with thresholds, files that must exist, lines that must
  appear). A stage that does not reduce to a success criterion is not a stage.
- The goal is the agent's alone and about itself only: no step, choice,
  upload or approval of the owner's inside it; what is the owner's (push,
  tag, release, purchase) appears only as a prohibition. Nothing about what
  follows the goal, no proposal to archive it; the file is never edited,
  renamed or moved by the agent once written.
- The time guides are budgets against over-running, not reasons to stop:
  the owner stops the work.
- Closure by record ("Закрыта записью") is for the impossible, and the
  impossible is only what the part's file names as a blocker (no board, a
  container that will not start after two repairs). Effort, a marginal
  threshold, or a shortage of flights are not blockers; a stop hook reads
  the closure formula literally (2026-10-30), so write it literally.

## 3. The Working Loop

Read → verify → decide → build the minimum → gate → commit → measure →
record → report. In practice:

1. **Read before planning**: the specification whole, the roadmap item, the
   register rows the work touches, the journals of the previous attempt. A
   recalled memory names a file or a flag: verify it still exists.
2. **Decide and write the decision down** in the deviations file the moment
   it is taken, with "required / why not / done / why so"; a doubt goes to
   the suspicious list the moment it appears and is marked "снято: how" when
   resolved, never deleted. A doubt noticed in the second hour is otherwise
   lost by the compactions before the report.
3. **Build the smallest change**: edit existing code before adding modules,
   types or parameters; nothing "for extensibility"; no configuration option
   that was not asked for; constants carry the measurement they came from in
   a comment; every new entity is listed in the report with the requirement
   that justifies it.
4. **Gate, then commit**: `./tools/gates.sh` (format, build, unit tests,
   script tests, quality; `X=0`), `--docs` for Markdown alone; in the
   background with polling when it runs longer than a terminal call allows.
   One commit per completed change, on the current branch.
5. **Measure in the simulator** by the series rules (section 5), one flight
   at a time, never two heavy processes beside each other.
6. **Record in the same commit** (section 4) and **report** (section 8).

Debt met on the way is repaired at once with its own commit and test; what
is too big goes to the register with its class (a: very hard, b: a deep
rework, c: the owner's decision) and its options. Scope is not widened: a
tempting neighbouring item is named in the report, not done.

## 4. The Registers: Where The Truth Of The Project Lives

- `docs/specification.md`: the requirements (R), the acceptance rules (A),
  the facts of the flights (F), the measured constants and laws (K), the
  invariants (I), each marked Owner or Agent with its date and source. A
  change lands in the same commit as the code, with a change-log row that
  carries the justification and the hash of its own commit (commit, then
  `sed -i "s/| this commit |$/| HASH |/"`, a docs gate, a stamp commit;
  `grep -c "this commit"` is 0 at the end of every part). Owner rows change
  only by the owner's word or by the standing permission to decide with a
  record (2026-10-24), named in the row.
- `docs/roadmap.md`: the items with their types, prerequisites, stages and
  completion criteria; a done item gets its "Done on" record with the
  numbers and the runs, then moves to the Completed section with a closing
  summary; numbers are never reused.
- `docs/technical_debt.md`: what is known wrong and set aside, each entry
  with its class, its numbers and its options; the attempts to return to the
  target mean speed (R3) have their own table; an entry leaves when repaired.
- `CHANGELOG.md`: the Unreleased section grows with every piece of work; the
  release sections are the owner's.
- The journals under `log/tools/journal/` (a `.keep` directory): the time
  with its offset, the hypothesis before every flight or stand, the numbers
  after, the decisions with their numbers, the voided flights with their
  reasons, the estimate of the time left after every stage.
- The deviations file beside the goal (both parts: deviations, suspicious).
- The agent's memory: what is not in the repository and matters next time
  (the owner's rules with their reasons, the state of the work, pointers),
  one fact a file, updated when a rule changes, never the repository's own
  content.

A figure in a document is reproducible from the repository: the script that
produced it is under git in `tools/` with a line in `tools/README.md`, never
in `log/`. Anything in `log/` worth more than a week lives in a directory with
a `.keep` file; the weekly pruning deletes the rest.

## 5. Flights: How A Measurement Is Made

- The acceptance is five single flights on the stereo set and five on the
  lidar, on one commit, each read before the next (A1); a code change after
  a series means the series again for the groups it touches.
- Flights are flown one at a time through `tools/fly_until_valid.sh` (and
  `tools/series2.sh` under it): the flight's core set (A4), the real-time
  factor 1.0, the imperfections profile on (F21), the quiet-host gate and the
  watchdog (`tools/rtf_watchdog.py`) around the counted flight. A flight the
  host's load slowed under the verdict's floor (A7) is void and flown again
  under the next run number; a counted flight that fails a check is a finding
  and is never flown again to make it pass.
- A run the user starts by hand (`./scripts/sim_*.sh`, the `sim-*` targets)
  never waits for the host (the owner, 2026-10-30): the gate belongs to the
  series tools alone.
- Nothing heavy runs beside a flight: no build, no replay, no analysis
  (an analysis beside r2161 voided it). Between flights, everything.
- A finding is traced to its cause with the tools that read the records:
  `tools/estimate_error.py` (the estimate against the truth, the frame's
  turn), `tools/node_budget.py` (the ticks and the ages), `resources.csv`
  (the cores, the GPU, the real-time factor), `tools/body_truth_clearance.py`
  (the true clearance). A cause is separated by an A/B on the same day on the
  same host (the clean vehicle against the profile, one engine against
  another), never by a guess; eight against five flights with a spread of
  0.10 m/s is "suggestive", and is written so.
- The simulator's truth is read only by the checks and the analysis tools,
  never by the onboard code; the onboard code knows no location, zone or
  coordinate (a held-out world must be flyable with nothing adapted).
- The navigation invariants hold in every change: unknown space is
  traversable at no cost (I1); a prohibition comes only from a measurement
  and is lifted by one (I2); no latch on history (I3); no decay by time (K10).
  A disputed point takes the most conservative option that passes the
  acceptance, and a line in the report.
- The CPU rule (the owner, 2026-10-24): nothing heavier in the onboard loop
  without its measured cost; above the base plus five percent, roll back or
  register the cost with a decision.

## 6. Code And Repository Discipline

- The container workflow only (`scripts/container_run.sh`,
  `scripts/dev_shell.sh` without `-c`); environment variables reach the
  container through its whitelist and the run's manifest, both with a test.
- C++20 everywhere; a source stays under 1000 non-empty lines (a new theme
  goes to a new module when a file sits at the limit); public headers compile
  alone (the self-containment check compiles every header under `src/`, so a
  header with an external include breaks the onboard image, 2026-10-30).
- Git: `git add` by explicit file paths only, never `-A`, `.`, `-u`, a
  directory or `commit -a`; no amend, squash, branches or trailers; never
  push, tag or release; one commit per finished change with a message that
  says what and why in the project's words.
- Tests follow the change: a unit test for a law, a script test for a
  contract (a whitelist, a manifest, a document's wording the tools rely on),
  a parity test for a second implementation of the same model.
- A port of a model to another backend keeps one body where the language
  allows (the shared host-device rollout body of item 28) and a parity test
  where it does not (the GLSL body against the C++ one, 1.4e-6 m/s^2).

## 7. Running Long Work On A Shared Host

- A terminal call is bounded (ten minutes here): gates, flights, builds and
  downloads run in the background with a log that states the start, the
  total and the progress, and are polled; a wrapper never cuts such a log
  down to its last line.
- Batch work uses the hardware: `--jobs $(nproc)`, independent items in
  parallel, the load checked with `top`; within memory, never squeezing the
  owner's own processes, which are left alone.
- Killing by pattern kills the shell that holds the pattern: a `pgrep -f` or
  `awk` over the process table that names a script also matches the command
  that runs the search (exit 144, several times in October 2026). Kill in a
  separate command, with a bracketed pattern (`[s]eries2.sh`), and never
  name the script in the same command line.
- Downloads land in their own directory (`/data/gazebo/` for heavy data);
  scripts that read downloaded data run with `python3 -I`; nothing is
  executed from inside a downloaded tree.
- Credentials come from the keyring, never from a prompt, a file, a log or a
  memory; remote systems are read-only unless the owner names a host and an
  action.

## 8. Reporting

The report is read by someone who did not watch the work. It leads with the
outcome and what could not be verified, gives the numbers in tables, names
every new entity with its requirement, lists the voided flights and the
time per stage, and ends with two sections that are never left out:

- **Отклонения от оригинальной постановки**: every item as required / why
  not / done / why so, or "none" only after comparing the result with the
  task's text.
- **Список подозрительного для проверки пользователем**: what a person must
  look at by eye, always present even when empty: inherited behaviour,
  duplicated mechanisms, states unknown until a later event, side repairs,
  paths covered only by unit tests, decisions without an owner, what was
  checked only by documentation.

A failed test, a voided flight or a skipped step is said plainly with its
output; a claim of completion is made only when every criterion is met or
its closure is written with numbers and options.

## 9. Autonomy: What The Agent Decides, And What It Does Not

- The owner's standing permission (2026-10-24): decide in the owner's place,
  always with a record (the register row, the deviations file, the
  journal). A newer rule wins over an older one, and the conflict is reported
  the moment it is noticed.
- No questions and no pauses during a goal: an assumption is stated and the
  work goes on; what only the owner can decide is written as a class (c)
  entry and the work continues around it.
- Skills and procedures run only when invoked by name; a request that sounds
  like one is not an invocation.
- After a compaction: the goal file whole, the current part whole, the
  journals, the specification, then the work; never the memory alone.
- A stop hook or a reviewer that reads a criterion literally is right to:
  the criterion is met, or its closure is written in the words the file
  allows, or the work is honestly reported as not closed. No looping on the
  same claim, no cherry-picking of flights, no tuning to pass.
- Without hardware, everything that can be done without it is done (a
  backend on the host's GPU, an arm64 image under emulation, a board's
  documentation read layer by layer with the pages that answered 404 named);
  what needs the board is closed by record with the owner's options.

## 10. Lessons, Dated

- 2026-09-19: two requirements only; every other metric a NOTE, never a FAIL.
- 2026-09-25: the speed is measured on the simulation clock; a flight under
  foreign load is never counted.
- 2026-10-07: a flight is watched while it flies and stopped when the load
  slows it (the watchdog), then flown again.
- 2026-10-18: a goal with a step "by the owner's command" cannot be closed by
  the agent; the goal is the agent's alone.
- 2026-10-19: a doubt is written the moment it appears, or the compaction
  eats it.
- 2026-10-24: "the expected speed is the real one"; the flight's core set;
  the permission to decide with a record; the goal is about itself only and
  its file is never touched after the work.
- 2026-10-25: a density written as a standing deviation gave a bias fourteen
  times its source and three crashes; read the unit of every model parameter
  before writing the number (gz-sensors `dynamic_bias_stddev`).
- 2026-10-29: a world-control step without `pause=true` in the same request
  unpauses the world; a chain's last `[[ ]] && cmd` under `set -e` ends the
  run; a relay in Python cost a third of a core and voided camera flights,
  the same in C++ cost nothing.
- 2026-10-30: the stop hook reads "closed by record" literally: only the
  named blockers close a stage; a header with an external include breaks the
  onboard image through the self-containment check; the user's runs never
  wait for a quiet host; the target mean speed with its 20 percent and the
  debt entry for every series under it.
