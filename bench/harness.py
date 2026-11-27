# 실험 진행 공통 코드

import csv
import os

from engine import config
from engine.profiler import Profiler
from engine.scene import Scene
from bench import metrics

RESULTS_DIR = os.path.join(os.path.dirname(__file__), "results")

class ClassSceneSim:
    def __init__(self, bodies, gravity=config.GRAVITY):
        self.scene = Scene(bodies, 0)
        self.bodies = self.scene.bodies
        self.gravity = gravity
        self.profiler = Profiler()

    def step(self, dt):
        p = self.profiler
        p.begin_step()
        with p.section("integrate"):
            for b in self.bodies:
                if not b.is_static:
                    b.velocity.y -= self.gravity * dt
            self.scene.update_position(dt)
        with p.section("collide"):
            self.scene.handle_collisions()
        n = len(self.bodies)
        p.set("pair_checks", n * (n - 1) // 2)
        p.end_step()

def run(sim, seconds=3.0, window=0.5, dt=config.DT):
    steps = int(round(seconds / dt))
    win = int(round(window / dt))
    rest = []
    for s in range(steps):
        sim.step(dt)
        if s >= steps - win:
            rest.append(metrics.rest_ratio(sim.bodies))
    p = sim.profiler
    step_ms = p.mean("step_ms", win)
    return {
        "step_ms": step_ms,
        "budget_pct": step_ms / config.FRAME_BUDGET_MS * 100.0,
        "collide_ms": p.mean("collide_ms", win),
        "pair_checks": p.mean("pair_checks", win),
        "rest_ratio": sum(rest) / len(rest),
    }

def write_csv(name, rows):
    os.makedirs(RESULTS_DIR, exist_ok=True)
    path = os.path.join(RESULTS_DIR, name)
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader()
        for r in rows:
            writer.writerow({k: round(v, 4) if isinstance(v, float) else v for k, v in r.items()})
    return path