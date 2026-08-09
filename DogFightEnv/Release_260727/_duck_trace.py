import sys
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parent
for p in (ROOT, ROOT / "src"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))
from _aim_time_probe import DT, make_state
from _degenerate_probe import ScriptedProvider
from DogFightEnvWrapper import DogFightWrapper
from dogfight.ai.bt_action_provider import BTActionProvider

DLL = sys.argv[1] if len(sys.argv) > 1 else "AIP_DCS.dll"
SEED = int(sys.argv[2]) if len(sys.argv) > 2 else 0
rng = np.random.default_rng(SEED)
own, tgt = make_state(rng)
print(f"[{DLL} seed{SEED}] own=", np.round(own,1), " tgt=", np.round(tgt,1))
env = DogFightWrapper(
    env_config={"observation_mode": "tactical16", "ownship_control_mode": "rl",
                "target_mode": "rl", "max_engage_time": 200.0, "min_altitude": 300.0,
                "ownship": own, "target": tgt, "initial_scenario": {"mode": "default"}},
    ownship_action_provider=BTActionProvider(dll_name=DLL),
    target_action_provider=ScriptedProvider("duck"))
env.reset(seed=SEED)
term = trunc = False
step = 0
while not (term or trunc):
    _, _, term, trunc, _ = env.step(np.zeros(4, dtype=np.float32))
    step += 1
    if step % 600:   # 10s
        continue
    g, o, t = env._geo_info, env._ownship_state, env._target_state
    d = float(g._get_distance(o, t))
    ma = abs(float(g._get_antenna_train_angle(o, t, False)))
    print(f"TRACE t={step*DT:6.1f} d={d:7.0f} ATA={ma:6.1f} | "
          f"me alt={-o[2]:6.0f} spd={o[6]:5.1f} rol={o[3]:7.1f} pit={o[4]:6.1f} hdg={o[5]:6.1f} | "
          f"duck alt={-t[2]:6.0f} spd={t[6]:5.1f} rol={t[3]:7.1f} pit={t[4]:6.1f} hdg={t[5]:6.1f}",
          flush=True)
env.close()
