from engine import config
from engine.body import Rectangle
from engine.world import World
from engine.sleeping import SleepSettings
from bench import metrics
from bench.harness import write_csv
from bench.scenarios import make_ground, pile, tower, GROUND_TOP, BOX

_n = SleepSettings.naive()  # 단계마다 하나씩 켠다
STAGES = [
    ("OFF", SleepSettings.off()),
    ("NAIVE", _n),
    ("+D1 timer", _n.but(mode="IMPROVED", use_timer=True)),
    ("+D2 wake rule", _n.but(mode="IMPROVED", use_timer=True, wake_rule="moving")),
    ("+D3 chain wake", _n.but(mode="IMPROVED", use_timer=True, wake_rule="moving", chain_wake=True)),
    ("+D4 islands", SleepSettings.improved()),
]

def run_for(world, seconds):
    for _ in range(int(round(seconds / config.DT))):
        world.step(config.DT)

# 한 칸 올라갈 때마다 shift px씩 오른쪽으로 비켜 쌓은 탑 → 천천히 기울어 넘어짐
def tilted(k, shift):
    bodies = tower(k)
    for i, b in enumerate(bodies[1:]):
        b.center.x += i * shift
    return bodies


# 장면 1: 바닥의 상자 8개를 서로 다른 속도로 위로 던짐 (최고점에서 속도가 0이 됨)
def thrown(ss):
    bodies = [make_ground()]
    for i in range(8):
        b = Rectangle(x=100 + 60 * i, y=GROUND_TOP + BOX / 2, width=BOX, height=BOX, bounce=0.2)
        b.velocity.y = 150 + 50 * i
        bodies.append(b)
    w = World(bodies, sleep=ss)
    run_for(w, 3.0)
    return w, {"floating": metrics.floating_asleep(w.bodies)}


# 장면 2: 4단 탑을 3초 재운 뒤 맨 아래 상자를 빼고 2초 더
def remove_support(ss):
    w = World(tower(4), sleep=ss)
    run_for(w, 3.0)
    asleep = sum(1 for b in w.bodies if b.asleep)
    w.remove(w.bodies[1])
    run_for(w, 2.0)
    return w, {"asleep_before": asleep, "floating": metrics.floating_asleep(w.bodies)}


# 장면 3: 상자 n개를 떨어뜨려 5초 정착 (비용과 깜빡임)
def settle(ss, n=80):
    w = World(pile(n), sleep=ss)
    run_for(w, 5.0)
    p = w.profiler
    last = int(round(1.0 / config.DT))
    return w, {"mean_ms": p.mean("step_ms"), "last1s_ms": p.mean("step_ms", last),
               "sleep_events": w.sleep.events["sleep"], "wake_events": w.sleep.events["wake"]}


# 장면 4: 8단 기운 탑이 6초 동안 넘어지는 모양
def tilted_tower(ss):
    w = World(tilted(8, 3), sleep=ss)
    run_for(w, 6.0)
    return w, {"asleep": sum(1 for b in w.bodies if b.asleep)}


SCENARIOS = [("thrown", thrown), ("remove", remove_support), ("settle", settle), ("tilted", tilted_tower)]


def main():
    for sname, fn in SCENARIOS:
        rows = []
        ref_w, ref_m = fn(SleepSettings.off())   # 정답
        for cname, ss in STAGES:
            if ss.mode == "OFF":
                w, m = ref_w, ref_m              # OFF는 정답을 재사용
            else:
                w, m = fn(ss)
            mean_err, max_err = metrics.position_error(w.bodies, ref_w.bodies)
            row = {"scenario": sname, "stage": cname, **m, "err_mean": mean_err, "err_max": max_err}
            rows.append(row)
            print("  ".join(f"{k}={v:.2f}" if isinstance(v, float) else f"{k}={v}" for k, v in row.items()), flush=True)
        print("저장:", write_csv(f"phase7_{sname}.csv", rows))


if __name__ == "__main__":
    main()