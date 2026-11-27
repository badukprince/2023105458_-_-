# 수업 Scene 코드 수정

from engine import config
from engine.collision import detect_contact     
from engine.profiler import Profiler            
from engine import solver            

from dataclasses import dataclass, replace
from engine.sleeping import SleepSystem, SleepSettings

# 물리 설정 묶음. 항목을 하나씩 켜고 끄며 효과를 따로 측정한다.   
@dataclass
class PhysicsSettings:
    iterations: int = config.SOLVER_ITERATIONS
    restitution_threshold: float = config.RESTITUTION_THRESHOLD
    friction: bool = True
    position_correction: str = "projection"  # "class"(100% 즉시) | "projection"(percent, slop)
    percent: float = config.POSITION_PERCENT
    slop: float = config.POSITION_SLOP
    clipping: bool = True
    warm_start: bool = True
    match_dist: float = config.WARM_START_MATCH_DIST

    @staticmethod
    def class_like():  # 수업 엔진 방식
        return PhysicsSettings(iterations=1, restitution_threshold=0.0, friction=False,
                               position_correction="class", clipping=False, warm_start=False)

    def but(self, **kw):  # 일부만 바꾼 복사본
        return replace(self, **kw)


class World:
    def __init__(self, bodies=None, settings=None, gravity=config.GRAVITY, sleep=None):
        self.bodies = list(bodies) if bodies else []
        self.settings = settings or PhysicsSettings()
        self.gravity = gravity
        self.contacts = []
        self.profiler = Profiler()
        self.warm_cache = {} # 지난 스텝 접촉점별 충격량
        self.sleep = SleepSystem(sleep or SleepSettings.off()) # 기본은 휴면 끔

    def add(self, body):
        self.bodies.append(body)

    def remove(self, body):
        self.bodies.remove(body)
        self.sleep.on_remove(body)

    def body_at(self, x, y):
        # 점 (x, y)를 포함하는 동적 물체 (나중에 넣은 것부터), 없으면 None
        for b in reversed(self.bodies):
            if b.is_static:
                continue
            if b.shape_type == "Circle":
                dx, dy = x - b.center.x, y - b.center.y
                if dx * dx + dy * dy <= b.radius * b.radius:
                    return b
            else:
                vs = b.get_vertices()
                crosses = []
                for i in range(len(vs)):
                    p, q = vs[i - 1], vs[i]
                    crosses.append((q.x - p.x) * (y - p.y) - (q.y - p.y) * (x - p.x))
                if all(c >= 0 for c in crosses) or all(c <= 0 for c in crosses):
                    return b
        return None

    def detect(self): # 접촉 모으기
        contacts = []
        checks = 0
        n = len(self.bodies)
        for i in range(n-1):
            a = self.bodies[i]
            for j in range(i+1, n):
                b = self.bodies[j]
                if solver.frozen(a) and solver.frozen(b):
                    continue
                checks += 1
                hit = detect_contact(a, b, self.settings.clipping)
                if hit is None:
                    continue
                if a.asleep:
                    self.sleep.on_touch(a, b)
                elif b.asleep:
                    self.sleep.on_touch(b, a)

                contacts.append(solver.Contact(a, b, *hit))

        self.profiler.set("pair_checks", checks)
        self.profiler.set("contacts", len(contacts))
        return contacts

    def integrate_velocity(self, dt):
        for body in self.bodies:
            if not solver.frozen(body):
                body.velocity.y -= self.gravity * dt

    def integrate_positions(self, dt):
        for b in self.bodies:
            if not solver.frozen(b):
                b.center.x += b.velocity.x * dt
                b.center.y += b.velocity.y * dt
                b.angle += b.angular_velocity * dt 

    # 반암시적 오일러 사용!
    def step(self, dt=config.DT):
        s = self.settings
        p = self.profiler
        p.begin_step()

        with p.section("detect"):
            self.contacts = self.detect()

        with p.section("integrate"):
            self.integrate_velocity(dt)

        with p.section("solve"):
            solver.prepare(self.contacts, s)
            if s.warm_start:
                solver.warm_start(self.contacts, self.warm_cache, s)
            solver.solve_velocities(self.contacts, s)
            if s.warm_start:
                self.warm_cache = solver.store_cache(self.contacts)
            else:
                self.warm_cache = {}

        with p.section("integrate"):
            self.integrate_positions(dt)

        with p.section("correct"):
            if s.position_correction == "class":
                solver.correct_positions_class(self.contacts)
            else:
                solver.correct_positions_projection(self.contacts, s)

        with p.section("sleep"):
            self.sleep.update(self.bodies, dt, self.contacts) # 휴면 판정은 스텝 끝 (속도가 다 정해진 뒤)
        p.set("awake", sum(1 for b in self.bodies if not solver.frozen(b))) # 깨어 있는 물체 수
                
        p.end_step()