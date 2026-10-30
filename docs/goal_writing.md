# Writing A Goal: The Brief For An Unattended Session

A goal is the one document an agent has when it wakes up in the middle of a
ten-hour session after its context was compacted. It has to answer, with no
other source: what is being done and why, what is already true, what is
forbidden, what each stage is and how to tell it is done, where to write,
when to stop and what to report. This is the method the goals of this
project were written by (the files of 2026-10-24, 2026-10-29 and 2026-10-30
are the examples; the owner keeps them outside the repository and archives
them). It is written for any agent and any owner; the project's own rules
it leans on are in [`autonomous_development.md`](autonomous_development.md)
and [`specification.md`](specification.md).

## Where Goals Live

One active goal a working folder, outside the repository (the owner names
the folder; here `~/Downloads/gazebo/`). A finished goal is archived by the
owner, never deleted and never by the agent: the agent does not rename,
copy, move, delete or edit the file once the work is done (an agent that
renamed its own goal left the stop hook with no goal to read and it looped;
2026-10-24). Before writing a new goal, read the previous ones in that folder,
the last one whole: they carry rules, baselines and names the new goal must
keep. A goal of several parts is one root file with the common rules and one
file a part in a folder beside it.

## The Procedure

1. **Collect the statement.** Everything the owner said about the task, in
   the owner's words; the owner's explicit decisions apart from everything
   else. Reread the standing rules (the agent's memory, the repository's
   instructions): every rule that applies goes into the goal explicitly,
   because the goal is read before the memory and must not contradict it.
2. **Verify the facts now.** Every fact in the goal is checked at writing
   time: `path:line` from the current checkout, commit hashes, branch names,
   versions, tool availability, data locations and sizes, baseline numbers
   from earlier runs with the runs they came from. Nothing is written from
   memory of an earlier conversation. Mark what is verified, what is
   inferred, what is unknown.
3. **Close the open questions.** The goal is executed without the owner.
   Each open question gets a decision by the agent with a one-line reason,
   or an explicit instruction how to decide during the work (what to
   measure, what counts as evidence), never "ask the owner".
   - The goal is the agent's alone: no stage, sub-step, checklist line,
     success line or definition of "done" may depend on the owner doing,
     choosing, sending, uploading, reviewing or commanding anything. An
     action reserved for the owner (push, tag, release, a purchase, a post)
     is outside the goal and appears only as a prohibition; the goal ends
     with the last thing the agent can do alone (drafts in a named folder,
     every candidate measured, the table written). A choice the owner wants
     to make personally is handled by preparing every candidate, or by a
     decision rule the agent applies; the owner chooses later, outside the
     goal (2026-10-18: a release goal with "steps by the owner's command"
     could not be closed by the agent).
   - The goal is about itself only: no queue of what follows it, no next
     goal by name, no release that comes after, no owner's later steps, not
     in the header, the "why", the decisions, the facts or the report. What
     comes after the goal is said once in the reply when the goal is written
     and once more when it is done; it never enters the file (2026-10-24: a
     "queue after this goal" line was read by the stop hook as unfinished
     steps and it looped).
   - Deviations from the original statement go into a deviations file beside
     the goal, created at writing time with the deviations already decided
     (one numbered item each: in the task / why not / done / why / who
     decided, when), and the goal orders the agent to append every new
     deviation the moment it is decided. The file's second part, the
     suspicious list, takes every doubt the moment it appears (what, where,
     why it is suspicious) and keeps it marked "resolved: how" instead of
     deleting it; a doubt noticed in the second hour is otherwise lost by the
     compactions before the report (2026-10-19). The same file feeds the
     final report.
4. **Draft the file** with the sections below, run the self-check, write it.
5. **Do not start executing.** Report where the file is, its stages in one
   line each, the decisions taken at writing time, and what the owner has to
   do before the start (free the machine, provide an environment), which is
   usually nothing.

## The Required Sections, In Order

1. **Header block** (ruled): `GOAL:` one or two lines naming the outcome, not
   the activity; the date and the owner's statement it was composed from;
   "the owner is away: work autonomously, without questions or pauses";
   "after any compaction, reread this file whole, the current part whole, the
   journals and the key documents"; previous goals named "for reference
   only". No line about what follows the goal.
2. **Main rule banner**: the one constraint whose violation is irreversible
   or outward-facing (only local work, nothing pushed, nothing published,
   remote systems read-only, the production default untouched). Always
   present, even when it repeats a standing rule.
3. **Why**: in plain words, what is broken or missing, what the owner gets at
   the end, what each check catches that the tests do not. A reader who knows
   nothing must understand why the stages exist.
4. **Repository, branches, commits**: which repository, which branch (and the
   check that the agent is on it, "blocked" otherwise), the gate that every
   commit needs, what is never committed, where the working folder and the
   bulk data live, the size limits of sources.
5. **Decisions, not to be reopened**: the owner's decisions with "(the owner,
   date)"; the agent's decisions taken at writing time with the reason in one
   line, numbered so the journal can refer to them.
6. **What is true on the date (verified)**: the facts from step 2 with
   `path:line`, hashes, numbers, data paths, what the environment has and
   lacks. This is the part that saves the most time after a compaction.
7. **Hard limits**: the forbidden list (push, tags, releases, branches,
   amend and squash, subagents, pattern kills, `git add` by directory or
   `-A`, trailers, the owner's own processes and configurations, heavy
   processes beside a measurement, cache cleaning); the hardware rule (batch
   work in parallel on all cores, the load checked, within memory); the
   logging rule for batch scripts (start, total, progress); remote access
   read-only by default with the allowed sources listed and any permitted
   mutation named with host and action; credentials only from a keyring,
   never in the goal, the journal or the memory; the code conventions that
   apply (minimal change, the list of new entities, no options for later);
   "no pauses and no questions to the owner"; the repair rule (a small defect
   met on the way is repaired with a test and its own commit, a big one is
   recorded with options and a recommendation, the scope is not widened).
8. **Stages**, numbered, each with a time guide in hours (a budget against
   over-running, never a reason to stop early), the order (strict, or which
   run in parallel), **Why** (one or two lines), **How** (concrete: commands
   with flags, paths, parameters, which tool from where, what to record),
   **Success** (measurable: numbers with thresholds, files that must exist,
   lines that must or must not appear, tests that must pass; "or explained in
   detail why not" only where a live check may be impossible). A stage that
   does not reduce to a success criterion is not a stage. A live check takes
   the time it needs and no more: never an hour of waiting where five
   minutes give the same answer (a shortened timeout, a prepared state).
9. **Recording, right after every step**: the journal (the time with its
   offset, the hypothesis before each run, the result, the decisions with who
   took them and why, the failures, the output paths, the time left after
   each stage); the registers and documents to update; the memory to write;
   the deviations file, both parts, at the moment a deviation is decided or a
   doubt arises or is resolved.
10. **Stop and report**: the definition of "done" (every stage's success
    criterion met or its closure written, by the agent's own work, never
    "waits for the owner") and of "blocked" (what to record, the options with
    a recommendation, what not to do instead); the report's sections (per
    stage what was done and the commits, not pushed; the new entities with
    the requirement for each; what was not verified and why; the time per
    stage); the two lists that are never left out, the deviations from the
    original statement (each as required / why not / done / why so, or
    "none" only after comparing the result with the statement) and the
    suspicious list for the owner's eyes (always, even when empty); the line
    "after the work, nothing is done with the goal file: not renamed, not
    copied, not deleted, not edited; its fate is the owner's"; and "before the
    word 'done', check every item of this section and of each part's".
11. **Standing warnings**: the recurring traps of the project (the unit of a
    model parameter, a configuration that overrides a default, the time
    format, the terms that must not be replaced, the version check before
    every run, the interpreter flags for downloaded data).

"Closed by record" is the way out of the impossible, and it has to be
spelled out: the part's file names the blockers that allow it (no board; a
service that will not start after two repairs), and only those; an unmet
criterion is otherwise reported as unmet. A stop hook or a reviewer reads the
formula literally, and should (2026-10-30).

## Style

- The owner's language for the text, plain text, ruled separators between
  sections, numbered items, about 110 columns; one file a part, no Markdown
  headings (plain text is what the agent reads after a compaction).
- Precise over polite: `path:line`, hashes, flags, thresholds, file names.
  "Check that it works" is not an instruction; "`grep -c '<line>' <log>` is
  at least 1 after <time>" is.
- Self-contained but not a copy of the documents: reference them by path and
  section. Typical length 150 to 300 lines a file; longer is fine when the
  facts need it, shorter is suspicious.
- Baselines: when a stage compares runs, the goal states the base numbers and
  the commit they came from.
- Nothing optional is left vague: either "if stages 1 to 4 are done" or in
  scope.

## The Self-Check Before Writing

- Every section above is present; every stage has Why, How and Success.
- Every path, hash, line number, tool and service was checked in this
  session.
- Every open question is closed by a decision or by an instruction how to
  decide; no "ask".
- Nothing waits for the owner: search the draft for "by the owner's
  command", "the owner sends", "the owner uploads", "the choice is the
  owner's", "after approval"; each hit is moved to the hard limits as a
  prohibition or rewritten as the agent's own last step; "done" is decidable
  by the agent alone from files, numbers and git.
- The goal is about itself only: search for "queue", "next goal", "after this
  goal", "then", "further", "after the release", "the owner's decision",
  "archive"; each hit that points outside the task is deleted.
- The main rule banner names the irreversible risk; remote access is stated
  read-only with the allowed sources; no credentials in the file.
- The forbidden list matches the standing rules in force; nothing in the goal
  contradicts the repository's instructions.
- The report section demands the two lists and the new-entity list, does not
  mention archiving, and carries the "nothing is done with the goal file"
  line; the deviations file exists beside the goal with both parts and is
  named in the recording section.
- Preconditions only the owner can satisfy (a branch, a machine) are stated
  as a stop condition and told to the owner in the reply.
