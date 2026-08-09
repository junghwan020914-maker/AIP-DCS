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

class Fixed(ScriptedProvider):
    def __init__(self, thr):
        super().__init__("duck")
        self.thr = thr

rng = np.random.default_rng(0)
own, tgt = make_state(rng)
own = list(own); tgt = list(tgt)
own[2] = tgt[2] = -3000.0          # both at 3000 m
own[6] = tgt[6] = 204.0
env = DogFightWrapper(
    env_config={"observation_mode": "tactical16", "ownship_control_mode": "rl",
                "target_mode": "rl", "max_engage_time": 200.0, "min_altitude": 300.0,
                "ownship": own, "target": tgt, "initial_scenario": {"mode": "default"}},
    ownship_action_provider=Fixed(1.00),
    target_action_provider=Fixed(0.70))
env.reset(seed=0)
term = trunc = False
step = 0
while not (term or trunc) and step < 200 * 60:
    _, _, term, trunc, _ = env.step(np.zeros(4, dtype=np.float32))
    step += 1
    if step % 1200:   # 20s
        continue
    o, t = env._ownship_state, env._target_state
    print(f"THRUST t={step*DT:6.1f} | thr1.00 spd={o[6]:6.1f} alt={-o[2]:6.0f} pit={o[4]:5.1f} "
          f"| thr0.70 spd={t[6]:6.1f} alt={-t[2]:6.0f} pit={t[4]:5.1f} | diff={o[6]-t[6]:+6.1f}",
          flush=True)
env.close()
