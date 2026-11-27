# Step 4. What Safety Means Here

The vehicle flies through space nobody has cleared for it, by a map it is
still drawing, on an estimate of its pose that drifts. This step says what
keeps it from touching anything (R1), layer by layer, and where each rule is
written down.

## The Rule Under Everything: Stop Inside What You Have Seen

Unknown space is free to plan through (I1). What makes that safe is speed:
the vehicle never moves faster than lets it come to rest inside the space
its forward sensor has measured as clear (K9), along the direction it is
moving (K68) and through the column a climb or a descent sweeps (K69). A
wall that appears at the edge of the sensor's range is then always a wall
the vehicle can stop before. The stereo set sees a shorter way than the
lidar (K7), and flies slower for it.

Three things follow, and they are invariants, not tuning:

- a prohibition comes only from a measurement and is lifted by one (I2);
- nothing is forbidden because of the vehicle's history: no latch, no
  "this passage was tried" (I3);
- safety and the quality of the flight come before speed (I6).

## A Plan Is Flown Only As A Path That Ends At Rest

The planner hands over an execution horizon: a finite path that stops, with
its clearance checked against the memory and once more against the latest
frame of the sensor at the moment it is committed (K86). The offboard node
flies that and nothing else. If the planner dies, falls behind or simply
has no route, the vehicle does not coast: it is already on a path that
ends. A horizon also carries a promise of when the next one is due (K89);
a computer slower than the promise makes the vehicle slower, not blind.

## When Something Fails

The offboard node descends a ladder by what it has lost (K90): with the
planner gone it holds and then lands by itself; with the estimate gone it
holds level and descends level; and under the stack stands the autopilot's
own failsafe. A reset of the autopilot's position is flown on when small
(K1) and ends the navigation when large (K77). The vehicle judges its own
light (K12), knows darkness from unexplored space (F6), does not enter
dense smoke (R7, F19) and treats a warm body as something that may walk
(K66).

The stack can also fly as a mode of PX4's own, so that a pilot's switch
takes it away and gives it back as with any other mode (K91); the pilot
arms, the stack never does (K29).

## What Is Not Solved

The register of what is known to be wrong is
[technical_debt.md](../technical_debt.md), kept beside the code and as
honest as the tests. Among its rows today: the pose estimate drifts on a
first pass, and a place must be seen again to pull it back (K92); the lidar
set has no loop closure and needs one in tunnel worlds (K93); a vehicle
that comes to rest within a wall's clearance is not moved out of it.

## How A Claim Of Safety Is Checked

- Every flight's check judges contact and goal against the simulator's
  truth ([testing.md](../testing.md)); the truth is read by the checks and
  never by the flight code (I5).
- A change is accepted by five flights on each sensor set (A1), flown one
  at a time (A3) on a host that is not doing something else (A4, A7).
- After a fix, the flights are those the change acts on and then the
  acceptance, and no more (A14).
- A contact in a counted flight is traced to its cause and repaired; it is
  never flown again to make it pass
  ([autonomous_development.md](../autonomous_development.md)).

Next: [testing a change and reading a flight](05_testing_debugging.md).
