# 측정 지표

from engine import config
import math

def dynamic(bodies):
    return [b for b in bodies if not b.is_static]

def is_resting(body):
    #선속도와 각속도 모두 기준보다 작으면 정지
    v = body.velocity
    return (v.x * v.x + v.y * v.y < config.REST_LINEAR_SPEED ** 2 and abs(body.angular_velocity) < config.REST_ANGULAR_SPEED)

def rest_ratio(bodies):
    dyn = dynamic(bodies)
    if not dyn:
        return 1.0
    return sum(1 for b in dyn if is_resting(b)) / len(dyn)

# 받침 검사: 1px 내려 봤을 때 아래에서 닿는 물체가 있으면 받쳐져 있음
def is_supported(body, bodies, probe=1.0):
    from engine.collision import detect_contact
    body.center.y -= probe
    try:
        for o in bodies:
            if o is body:
                continue
            hit = detect_contact(o, body)
            if hit is not None and hit[0].y > 0.3:   # 법선(o → body)이 위쪽 = o가 아래에서 받침
                return True
        return False
    finally:
        body.center.y += probe


# 잠든 채 공중에 떠 있는 물체 수
def floating_asleep(bodies):
    return sum(1 for b in dynamic(bodies) if b.asleep and not is_supported(b, bodies))


# 두 시뮬레이션의 최종 위치 차이 (px): 평균, 최대. OFF를 정답으로 본다
def position_error(bodies, ref_bodies):
    errs = [math.hypot(a.center.x - b.center.x, a.center.y - b.center.y)
            for a, b in zip(bodies, ref_bodies) if not a.is_static]
    if not errs:
        return 0.0, 0.0
    return sum(errs) / len(errs), max(errs)