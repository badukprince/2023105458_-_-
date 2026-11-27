# 보고서 2.2 표 3행(모두 잠들 때까지)과 3.3 상자 수별 오차 분석

import sys; sys.path.insert(0, '.')
from engine import config
from engine.world import World
from bench.phase7 import STAGES
from bench.scenarios import pile
from bench import metrics

# 1) 상자 N개가 전부 잠들 때까지 걸린 시간 (단계별)
for n in (40, 80):
    out = []
    for name, ss in STAGES[1:]:
        w = World(pile(n), sleep=ss) 
        boxes = [b for b in w.bodies if not b.is_static]
        t_all = None
        
        for k in range(int(round(8 / config.DT))):
            w.step(config.DT)
            if all(b.asleep for b in boxes):
                t_all = (k + 1) * config.DT; break
        out.append(f"{name}={'-' if t_all is None else f'{t_all:.2f}'}")
    print(f"N={n} all-asleep:", "  ".join(out), flush=True)

# 2) N별 IMPROVED 오차 (OFF 기준, 5초)
for n in (10, 20, 40, 80, 160):
    ref = World(pile(n), sleep=STAGES[0][1])
    imp = World(pile(n), sleep=STAGES[-1][1])

    for _ in range(int(round(5 / config.DT))): ref.step(config.DT); imp.step(config.DT)
    mean, mx = metrics.position_error(imp.bodies, ref.bodies)
    print(f"N={n} IMPROVED err mean={mean:.3f} max={mx:.3f}", flush=True)
