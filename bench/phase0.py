# 수업 엔진으로 상자 N개를 3초 떨어뜨렸을 때!

from bench.harness import ClassSceneSim, run, write_csv
from bench.scenarios import pile

def main():
    rows = []
    for n in [10, 20, 40, 80, 160]:
        row = {"n": n, **run(ClassSceneSim(pile(n)))}
        rows.append(row)
        share = row["collide_ms"] / row["step_ms"] * 100
        print(f"N={n:4d}  step={row['step_ms']:7.2f} ms  예산 {row['budget_pct']:5.0f}%  "
              f"충돌 비중 {share:5.1f}%  검사 쌍 {row['pair_checks']:6.0f}  "
              f"정지 {row['rest_ratio'] * 100:5.1f}%", flush=True)
    print("저장:", write_csv("phase0.csv", rows))

if __name__ == "__main__":
    main()