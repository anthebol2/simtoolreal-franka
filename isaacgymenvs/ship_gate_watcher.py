"""Ship-gate watcher: flags when a v3 training run is "good enough" to harvest.

Criterion (the training's own mastery definition, reached by the successful v1
run): the tolerance curriculum has tightened success_tolerance to its 0.01
target AND the policy sustains mean successes >= 3.0 per episode at that
tolerance for `sustain` consecutive log blocks.

On trigger it:
  * writes SHIP_READY into the run directory (with the triggering stats)
  * sends a W&B alert from a tiny side run "<experiment>_shipgate"
    (job_type=monitor) so you get the app/email notification
  * keeps watching (training is NOT stopped — harvest the best checkpoint
    whenever convenient)

Usage (started automatically by launch_all_v3.sh):
  python isaacgymenvs/ship_gate_watcher.py \
    --log train_dir/<project>/<experiment>/train.log \
    --wandb_project simtoolreal_baseline_training
"""

import argparse
import re
import time
from pathlib import Path

TOL_RE = re.compile(r"scalars/success_tolerance\s*:\s*([\d.]+)")
SUCC_RE = re.compile(r"^\s*successes\s*:\s*([\d.]+)", re.M)

TARGET_TOLERANCE = 0.0101  # curriculum target 0.01 (+ float slack)
SUCCESS_THRESHOLD = 3.0


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--log", required=True)
    p.add_argument("--wandb_project", default="simtoolreal_baseline_training")
    p.add_argument("--sustain", type=int, default=50,
                   help="consecutive qualifying log blocks required")
    p.add_argument("--poll_seconds", type=float, default=60.0)
    args = p.parse_args()

    log_path = Path(args.log)
    run_dir = log_path.parent
    ready_path = run_dir / "SHIP_READY"
    experiment = run_dir.name

    print(f"[ship_gate] watching {log_path} (tol<= {TARGET_TOLERANCE}, "
          f"successes>= {SUCCESS_THRESHOLD} x{args.sustain} blocks)")

    pos = 0
    streak = 0
    alerted = ready_path.exists()
    last_tol, last_succ = None, None
    while True:
        if log_path.exists():
            with open(log_path, "r", errors="ignore") as f:
                f.seek(pos)
                chunk = f.read()
                pos = f.tell()
            for block in chunk.split("Policy 0:"):
                tol_m = TOL_RE.search(block)
                succ_m = SUCC_RE.search(block)
                if not (tol_m and succ_m):
                    continue
                last_tol, last_succ = float(tol_m.group(1)), float(succ_m.group(1))
                qualifying = (
                    last_tol <= TARGET_TOLERANCE and last_succ >= SUCCESS_THRESHOLD
                )
                streak = streak + 1 if qualifying else 0
                if streak >= args.sustain and not alerted:
                    msg = (
                        f"{experiment}: SHIP READY — success_tolerance={last_tol}, "
                        f"successes={last_succ:.2f} sustained over {args.sustain} "
                        f"log blocks. Training continues; harvest "
                        f"nn/best or latest checkpoint and run the deployment-"
                        f"pipeline eval gate."
                    )
                    ready_path.write_text(msg + "\n")
                    print(f"[ship_gate] {msg}")
                    try:
                        import wandb

                        run = wandb.init(
                            project=args.wandb_project,
                            name=f"{experiment}_shipgate",
                            job_type="monitor",
                        )
                        wandb.alert(title=f"SHIP READY: {experiment}", text=msg)
                        run.finish()
                    except Exception as e:  # alert failure must not kill the watch
                        print(f"[ship_gate] wandb alert failed: {e}")
                    alerted = True
        time.sleep(args.poll_seconds)


if __name__ == "__main__":
    main()
