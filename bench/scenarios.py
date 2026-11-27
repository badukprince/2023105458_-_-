# 실험 장면

import math

from engine.body import Rectangle

GROUND_TOP = 40.0 #바닥 윗면 y
BOX = 24.0 # 박스 한 변 길이
GAP = 2.0 # 박스 사이 간격

def make_ground(width=2000.0, bounce=0.2):
    return Rectangle(x=400, y=GROUND_TOP / 2, width=width, height=GROUND_TOP, is_static=True, bounce=bounce, name="Ground")

# 상자 n개 배치해 떨어뜨리기
def pile(n, bounce=0.2, x0=200.0, y0=60.0):
    bodies = [make_ground(bounce=bounce)]
    cols = max(1, int(math.sqrt(n)))
    step = BOX + GAP
    for i in range(n):
        row, col = divmod(i, cols)
        bodies.append(Rectangle(x=x0 + col * step, y=y0 + row * step, width=BOX, height=BOX, mass=1, bounce=bounce))
    return bodies

# 상자 k개를 한 줄로 쌓기
def tower(k, bounce=0.2, x=400.0):
    bodies = [make_ground(bounce=bounce)]
    for i in range(k):
        bodies.append(Rectangle(x=x, y=GROUND_TOP + BOX / 2 + i * (BOX + 0.5), width=BOX, height=BOX, mass=1, bounce=bounce))
    return bodies
