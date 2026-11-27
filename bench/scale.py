# 보고서 3.2 표(수업 엔진 / OFF / IMPROVED 비교). 

import sys; sys.path.insert(0, '.')
from engine import config
from engine.world import World
from engine.sleeping import SleepSettings
from bench.harness import ClassSceneSim
from bench.scenarios import pile
from bench import metrics
def go(sim, sec=5.0):
    steps = int(round(sec/config.DT)); last = int(round(1.0/config.DT))
    for _ in range(steps): sim.step(config.DT)
    p = sim.profiler
    asleep = sum(1 for b in sim.bodies if getattr(b,'asleep',False))
    return p.mean("step_ms"), p.mean("step_ms", last), metrics.rest_ratio(sim.bodies), asleep
for n in [10, 20, 40, 80, 160]:
    for name, mk in [("class", lambda: ClassSceneSim(pile(n))), ("OFF", lambda: World(pile(n), sleep=SleepSettings.off())), ("IMPROVED", lambda: World(pile(n), sleep=SleepSettings.improved()))]:
        m, l, r, a = go(mk())
        print(f"N={n:3d} {name:8s} mean5s={m:8.2f} last1s={l:8.3f} rest={r*100:5.1f}% asleep={a}", flush=True)
