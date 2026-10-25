"""The flight's core set (specification A4, 2026-10-24): FLIGHT_CPUSET confines the whole flight container to a set of
the host's CPUs through docker's cpuset, the subsystem masks are derived from the CPUs the container may run on, the
set is recorded in the manifest, the series fly on it by default and the quiet-host gate reads the set's own CPUs."""

import pathlib
import subprocess
import unittest

REPOSITORY = pathlib.Path(__file__).resolve().parents[2]


def derive(cpu_list: str) -> str:
    """The subsystem masks the helper derives when the process may run on cpu_list."""
    script = (f"source {REPOSITORY}/scripts/simulation_runtime_helpers.sh; derive_subsystem_cpu_lists")
    return subprocess.run(["taskset", "--cpu-list", cpu_list, "bash", "-c", script],
                          capture_output=True, text=True, check=True).stdout.strip()


class FlightCpusetContract(unittest.TestCase):
    def test_container_is_confined_to_the_set_and_passes_it_in(self):
        run = (REPOSITORY / "scripts/container_run.sh").read_text()
        self.assertIn('cpuset_args=(--cpuset-cpus "${FLIGHT_CPUSET}")', run)
        self.assertIn('"${cpuset_args[@]}"', run)
        self.assertIn("  FLIGHT_CPUSET\n", run)

    def test_manifest_records_the_set(self):
        runtime = (REPOSITORY / "scripts/runtime_evidence_runtime.sh").read_text()
        self.assertIn("SMOKE_FACE FLIGHT_CPUSET; do", runtime)

    def test_launcher_derives_the_masks_from_the_allowed_cpus(self):
        launcher = (REPOSITORY / "scripts/run_drone_nav_sim.sh").read_text()
        self.assertIn("< <(derive_subsystem_cpu_lists)", launcher)
        self.assertNotIn('control_cpu_list="${CONTROL_CPU_LIST:-0-$((cpu_count / 2 - 1))}"', launcher)
        self.assertIn("cpuset='$(taskset -pc $$ | sed 's/.*: //')'", launcher)

    def test_masks_keep_the_whole_host_proportions(self):
        # The masks of a 16-CPU host before the set existed: 0-7, 4-15, 12-15.
        self.assertEqual(derive("0-15"), "0,1,2,3,4,5,6,7 4,5,6,7,8,9,10,11,12,13,14,15 12,13,14,15")

    def test_masks_follow_a_narrowed_set(self):
        self.assertEqual(derive("4-15"), "4,5,6,7,8,9 7,8,9,10,11,12,13,14,15 13,14,15")
        self.assertEqual(derive("2,3,8,9"), "2,3 3,8,9 9")

    def test_fewer_than_four_cpus_disable_the_masks(self):
        self.assertEqual(derive("0-2"), "")

    def test_series_fly_on_the_set_through_the_gate(self):
        series = (REPOSITORY / "tools/series2.sh").read_text()
        self.assertIn('export FLIGHT_CPUSET="${FLIGHT_CPUSET-0-11}"', series)
        self.assertIn("./scripts/quiet_host_gate.sh", series)
        self.assertNotIn("/proc/loadavg); sim=", series)

    def test_quiet_gate_reads_the_set_when_one_is_given(self):
        gate = (REPOSITORY / "scripts/quiet_host_gate.sh").read_text()
        self.assertIn('if [[ -n "${FLIGHT_CPUSET:-}" ]]; then', gate)
        self.assertIn("set_busy_percent", gate)
        self.assertIn("calm=$(( load <= 15 ))", gate)
        self.assertIn("/proc/loadavg", gate)

    def test_inspect_prints_the_set(self):
        self.assertIn("cpuset={o.get('FLIGHT_CPUSET')}", (REPOSITORY / "tools/mx_inspect.py").read_text())


if __name__ == "__main__":
    unittest.main()
