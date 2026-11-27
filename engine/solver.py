def frozen(body):
    return body.is_static or body.asleep

def inv_mass(body):
    return 0.0 if frozen(body) else 1.0 / body.mass

def inv_inertia(body):
    return 0.0 if frozen(body) else 1.0 / body.inertia

class ContactPoint:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.pn = 0.0 # 이번 스텝에서 누적 법선 충격량!
        self.pt = 0.0 # 이번 스텝에서 누적 마찰 충격량!

class Contact:
    def __init__(self, a, b, normal, depth, points):
        self.a = a
        self.b = b
        self.nx = normal.x
        self.ny = normal.y
        self.depth = depth
        self.points = [ContactPoint(p.x, p.y) for p in points]

def relative_velocity(body_1, body_2, p, dx, dy):  # response_with_rotation 상대속도 참고
    rvx = (body_2.velocity.x - body_2.angular_velocity * p.r_2y) - (body_1.velocity.x - body_1.angular_velocity * p.r_1y)
    rvy = (body_2.velocity.y + body_2.angular_velocity * p.r_2x) - (body_1.velocity.y + body_1.angular_velocity * p.r_1x)
    return rvx * dx + rvy * dy

def apply_impulse(body_1, body_2, p, impulse_x, impulse_y): # response_with_rotation 마지막 참고
    if not frozen(body_1):
        body_1.velocity.x -= impulse_x / body_1.mass
        body_1.velocity.y -= impulse_y / body_1.mass
        body_1.angular_velocity -= (p.r_1x * impulse_y - p.r_1y * impulse_x) / body_1.inertia
    if not frozen(body_2):
        body_2.velocity.x += impulse_x / body_2.mass
        body_2.velocity.y += impulse_y / body_2.mass
        body_2.angular_velocity += (p.r_2x * impulse_y - p.r_2y * impulse_x) / body_2.inertia

def prepare(contacts, s):
    for c in contacts:
        body_1, body_2 = c.a, c.b

        inv_m1, inv_m2 = inv_mass(body_1), inv_mass(body_2) #역질량 (얼마나 쉽게 밀리는지)
        inv_i1, inv_i2 = inv_inertia(body_1), inv_inertia(body_2) #역관성모멘트(얼마나 쉽게 도는지)

        nx, ny = c.nx, c.ny
        tx, ty = -ny, nx  # 접선 = 법선을 90도 회전

        c.e = max(body_1.bounce, body_2.bounce)  # 얼마나 잘튀나

        c.mu_s = (body_1.static_friction * body_2.static_friction) ** 0.5  # 정지 마찰
        c.mu_d = (body_1.dynamic_friction * body_2.dynamic_friction) ** 0.5 # 운동 마찰

        for p in c.points:

            p.r_1x, p.r_1y = p.x - body_1.center.x, p.y - body_1.center.y
            p.r_2x, p.r_2y = p.x - body_2.center.x, p.y - body_2.center.y # 무게중심에서 접촉점까지 벡터

            r_1_perp_n = p.r_1x * ny - p.r_1y * nx  # 법선 방향으로 밀면 얼마나 도는지
            r_2_perp_n = p.r_2x * ny - p.r_2y * nx

            denominator = inv_m1 + inv_m2 + r_1_perp_n ** 2 * inv_i1 + r_2_perp_n ** 2 * inv_i2 # 법선 방향으로 툭 쳤을 때 얼마나 쉽게 반응하는지 합계
            p.mass_n = 1.0 / denominator if denominator > 0 else 0.0 # 역수, "법선 속도를 1 바꾸려면 얼마나 세게 쳐야 하나"

            r_1_perp_t = p.r_1x * ty - p.r_1y * tx  # 접선 방향으로 쳤을 때 얼마나 도나
            r_2_perp_t = p.r_2x * ty - p.r_2y * tx

            denominator = inv_m1 + inv_m2 + r_1_perp_t ** 2 * inv_i1 + r_2_perp_t ** 2 * inv_i2 # 접선 방향 합계
            p.mass_t = 1.0 / denominator if denominator > 0 else 0.0 # 역수

            penetration_velocity = relative_velocity(body_1, body_2, p, nx, ny) # 해당 점에서 두 물체가 법선 방향으로 움직이는 속도 차이

            if penetration_velocity < -s.restitution_threshold:
                p.target_vn = -c.e * penetration_velocity  # 빠르게 부딪힘 → 반발
            else:
                p.target_vn = 0.0  # 느린 접촉은 튀지 않게 함

            p.pn = 0.0 # 밀어내는 힘 누적 합계
            p.pt = 0.0 # 마찰 힘 누적 합계

def solve_velocities(contacts, s):
    for _ in range(s.iterations):
        for c in contacts:
            body_1, body_2, nx, ny = c.a, c.b, c.nx, c.ny
            tx, ty = -ny, nx
            
            for p in c.points:
                # 법선: 목표 분리 속도가 되도록
                penetration_velocity = relative_velocity(body_1, body_2, p, nx, ny) # 현재 두 물체의 법선 방향 속도 차이
                j = p.mass_n * (p.target_vn - penetration_velocity) # 목표 속도가 되기 위해 필요한 힘

                new_pn = max(p.pn + j, 0.0) # 지금까지 준 힘에 이번 힘 더함

                j = new_pn - p.pn # 실제로 줄 힘
                p.pn = new_pn # 합계 저장

                apply_impulse(body_1, body_2, p, nx * j, ny * j) # 법선 방향으로 그만큼 침

                if s.friction:
                    slide_velocity = relative_velocity(body_1, body_2, p, tx, ty) # 물체가 미끄러지는 속도 차이
                    new_pt = p.pt - p.mass_t * slide_velocity  # 미끄러짐을 완전히 멈추는 값
                    limit_s = c.mu_s * p.pn # 정지 마찰로 버틸 수 있는 최대 힘

                    if new_pt > limit_s or new_pt < -limit_s:  # 정지 마찰 한계를 넘으면 미끄러짐
                        limit_d = c.mu_d * p.pn  
                        new_pt = limit_d if new_pt > 0 else -limit_d

                    jt = new_pt - p.pt # 실제로 줄 힘
                    p.pt = new_pt # 합계 저장
                    apply_impulse(body_1, body_2, p, tx * jt, ty * jt)  # 실제로 치기

def correct_positions_class(contacts): 
    for c in contacts:
        body_1, body_2 = c.a, c.b
        sx, sy = c.nx * c.depth, c.ny * c.depth  # 분리 벡터, 어느 방향으로 얼마나 밀어낼지
        if frozen(body_1):
            body_2.center.x += sx
            body_2.center.y += sy
        elif frozen(body_2):
            body_1.center.x -= sx
            body_1.center.y -= sy
        else:
            body_1.center.x -= sx / 2
            body_1.center.y -= sy / 2
            body_2.center.x += sx / 2
            body_2.center.y += sy / 2

def correct_positions_projection(contacts, s):
    for c in contacts:
        body_1, body_2 = c.a, c.b
        inv_m1, inv_m2 = inv_mass(body_1), inv_mass(body_2)

        if inv_m1 + inv_m2 == 0.0: # 고정 물체면 건너뜀 (둘 다 벽)
            continue

        excess = c.depth - s.slop # slop = 봐주는 깊이
        if excess <= 0.0: # 봐주는 한에서 겹쳤다면 그냥 둬서 떨림 방지
            continue

        mag = excess / (inv_m1 + inv_m2) * s.percent # 밀어낼 기본 세기

        body_1.center.x -= c.nx * mag * inv_m1
        body_1.center.y -= c.ny * mag * inv_m1 # 1번 법선 반대로 밀기

        body_2.center.x += c.nx * mag * inv_m2
        body_2.center.y += c.ny * mag * inv_m2 # 2번 법선 방향으로 밀기

def warm_start(contacts, cache, s):
    max_d2 = s.match_dist ** 2  # 거리 비교는 제곱으로 (루트 계산 생략)
    for c in contacts:
        old_points = cache.get((c.a, c.b))  # 지난 스텝 이 쌍의 기록
        if old_points is None:  # 지난 스텝엔 안 닿아 있었으면 건너뜀
            continue
        tx, ty = -c.ny, c.nx  # 접선
        for p in c.points:
            best = None
            best_d2 = max_d2
            for (ox, oy, pn, pt) in old_points:  # 지난 점들 중 제일 가까운 점 찾기
                d2 = (ox - p.x) ** 2 + (oy - p.y) ** 2
                if d2 < best_d2:
                    best = (pn, pt)
                    best_d2 = d2
            if best is None:  # 2px 안에 없으면 새 점이니 0부터
                continue
            p.pn, p.pt = best  # 누적 합계를 지난 값에서 시작
            apply_impulse(c.a, c.b, p, c.nx * p.pn + tx * p.pt, c.ny * p.pn + ty * p.pt)  # 법선 + 마찰 한 번에 치기

# 이번 스텝 결과를 다음 스텝용으로 저장
def store_cache(contacts):
    cache = {}
    for c in contacts:
        cache[(c.a, c.b)] = [(p.x, p.y, p.pn, p.pt) for p in c.points]
    return cache

