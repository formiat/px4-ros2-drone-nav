"""truth_from_run.py RUN_DIR OUT_DIR: truth.csv (stamp_s,x,y,z,qw,qx,qy,qz, world ENU, body FLU) from the run's gz_pose.csv."""
import sys
rows = [l.split(",") for l in open(sys.argv[1] + "/gz_pose.csv")]
with open(sys.argv[2] + "/truth.csv", "w") as f:
    f.write("stamp_s,x,y,z,qw,qx,qy,qz\n")
    for r in rows:
        f.write(",".join(r[:8]) + "\n")
