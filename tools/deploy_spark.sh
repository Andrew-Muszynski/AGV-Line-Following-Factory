#!/bin/bash
# Push the BRKGA solver to the DGX Spark and benchmark it against this laptop.
#
#   bash tools/deploy_spark.sh            # copy + probe the environment
#   bash tools/deploy_spark.sh --bench    # copy, probe, then time a solve
#
# PREREQUISITE, run once, needs the Spark password:
#   ssh-copy-id -i ~/.ssh/id_ed25519.pub muszy@10.10.52.141
#
# WHAT GOES: the solver package and its testbed wrapper. No run data, no
# virtualenv, no .git, no __pycache__ -- those either do not belong on a
# shared box or will not survive the architecture change.
#
# ARCHITECTURE NOTE. GB10 is Grace-Blackwell, i.e. ARM64, while this laptop is
# x86_64. Numba JIT and torch both need to be present and working FOR ARM
# there; the probe below reports what is actually installed rather than
# assuming the laptop's environment carried over. Expect to install torch and
# numba on the Spark before the benchmark will run.
set -o pipefail

SPARK_USER="${SPARK_USER:-muszy}"
SPARK_IP="${SPARK_IP:-10.10.52.141}"
DEST="${DEST:-vrp_rpd}"
HOST="$SPARK_USER@$SPARK_IP"

cd "$(dirname "$0")/.." || exit 1
SRC="external/vrp_rpd"
[ -d "$SRC" ] || { echo "missing $SRC"; exit 1; }

echo "target: $HOST:~/$DEST"
echo

if ! ssh -o BatchMode=yes -o ConnectTimeout=8 "$HOST" true 2>/dev/null; then
  echo "SSH key auth is not working yet. Run this once, it will ask for the"
  echo "Spark password:"
  echo
  echo "    ssh-copy-id -i ~/.ssh/id_ed25519.pub $HOST"
  echo
  exit 1
fi
echo "ssh: key auth OK"

echo
echo "copying solver..."
ssh "$HOST" "mkdir -p ~/$DEST" || exit 1
tar --exclude='__pycache__' --exclude='*.pyc' --exclude='.git' \
    --exclude='*.json' --exclude='runs' --exclude='data' \
    -cz -C "$SRC" vrp_rpd agv_testbed \
  | ssh "$HOST" "tar xz -C ~/$DEST" || exit 1
for f in main.py eval_soln.py statistical_analysis.py run_alns.py; do
  [ -f "$SRC/$f" ] && scp -q "$SRC/$f" "$HOST:~/$DEST/"
done
echo "  done"

echo
echo "remote environment:"
ssh "$HOST" "cd ~/$DEST && python3 - <<'PY'
import platform, importlib
print('  arch       ', platform.machine())
print('  python     ', platform.python_version())
for m in ('numpy','torch','numba','pandas','scipy'):
    try:
        mod = importlib.import_module(m)
        extra = ''
        if m == 'torch':
            extra = f\"  cuda={mod.cuda.is_available()} devices={mod.cuda.device_count()}\"
            if mod.cuda.is_available():
                p = mod.cuda.get_device_properties(0)
                extra += f\"  {p.name} {round(p.total_memory/1e9,1)}GB\"
        print(f'  {m:11}', getattr(mod,'__version__','?'), extra)
    except Exception as e:
        print(f'  {m:11} MISSING ({type(e).__name__})')
PY"

if [ "$1" = "--bench" ]; then
  echo
  echo "benchmark: 6 agents, 2 resources, 300 generations, use_gp=False"
  ssh "$HOST" "cd ~/$DEST && python3 - <<'PY'
import sys, time
sys.path.insert(0, '.')
import numpy as np
from vrp_rpd.agv_testbed.grid_env import load_physical_grid
from vrp_rpd.agv_testbed.instance_builder import build_vrp_instance
from vrp_rpd.solver import VRPRPDSolver
inst = build_vrp_instance(load_physical_grid(rng=np.random.default_rng(101)), 6, 2)
t0 = time.time()
out = VRPRPDSolver(instance=inst, total_generations=300, gens_per_cycle=100,
                   use_gp=False, stagnation_limit=0, history_every=10).solve()
print(f'SPARK  makespan {out[\"makespan\"]:.2f}  [{out.get(\"source\")}]  wall {time.time()-t0:.1f}s')
PY"
  echo
  echo "laptop baseline for the same configuration was ~90 s (RTX 5060, makespan 71.00)."
  echo "Compare wall clock, and check 'source' is 'ga' on both -- a 'heuristic'"
  echo "result means the workers never started and the number is meaningless."
fi
