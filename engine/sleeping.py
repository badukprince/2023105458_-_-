from dataclasses import dataclass, replace

from engine import config

OFF, NAIVE, IMPROVED = "OFF", "NAIVE", "IMPROVED"

class Island:
    def __init__(self, bodies): #함께 잠든 물체 묶음
        self.bodies = bodies

class UnionFind:
    #a랑 b가 같은 묶음인가?
    def __init__(self):
        self.parent = {}

    def find(self, x):
        parent = self.parent
        root = x

        while parent.setdefault(root, root) != root:
            root = parent[root]
        while parent[x] != root:
            parent[x], x = root, parent[x]
        return root

    def union(self, a, b):
        ra, rb = self.find(a), self.find(b)
        if ra != rb:
            self.parent[ra] = rb

@dataclass
class SleepSettings:
    mode: str = NAIVE

    use_timer: bool = False # time_to_sleep 동안 계속 느려야 잠듦 (False: 즉시)
    wake_rule: str = "any" # "any"(닿으면 깨움) | "moving"(상대가 움직일 때만)
    chain_wake: bool = False # 깨거나 사라진 물체의 이웃(잠들 때 닿아 있던 물체)도 깨움
    islands: bool = False #  접촉으로 연결된 물체는 함께 잠들고 함께 깸

    linear_speed: float = config.SLEEP_LINEAR_SPEED # 속도가 이것보다 느리면 멈췄다고 봄
    angular_speed: float = config.SLEEP_ANGULAR_SPEED # 회전 속도가 이것보다 느리면 멈췄다고 봄

    time_to_sleep: float = config.TIME_TO_SLEEP
    wake_speed: float = config.WAKE_SPEED

    @staticmethod
    def off():
        return SleepSettings(mode=OFF)

    @staticmethod
    def naive():
        return SleepSettings(mode=NAIVE, use_timer=False, wake_rule="any", chain_wake=False, islands=False)

    @staticmethod
    def improved():
        return SleepSettings(mode=IMPROVED, use_timer=True, wake_rule="moving", chain_wake=True, islands=True)

    def but(self, **kw):
        return replace(self, **kw)

class SleepSystem:
    def __init__(self, settings=None):
        self.settings = settings or SleepSettings.off()
        self.events = {"sleep": 0, "wake": 0} # 누적 잠듦/깸 횟수 (측정용)

    def enabled(self):
        return self.settings.mode != OFF

    def is_slow(self, body): # 멈췄는지 검사
        s = self.settings
        v = body.velocity
        return (v.x * v.x + v.y * v.y < s.linear_speed * s.linear_speed and abs(body.angular_velocity) < s.angular_speed)

    def put_to_sleep(self, body, neighbors=(), island=None):
        
        # "자는 중" 표시
        body.asleep = True
        
        # 남아 있던 아주 작은 속도도 완전히 0으로 (꿈틀대지 않게)
        body.velocity.x = body.velocity.y = 0.0
        body.angular_velocity = 0.0
        body.island = island
        
        # 잠든 횟수 +1
        self.events["sleep"] += 1  

        # 닿아있던 물체끼리 서로의 이웃으로 기록
        for o in neighbors:
            body.sleep_neighbors.add(o)
            o.sleep_neighbors.add(body)

    def wake(self, body):
        body.sleep_timer = 0.0
        if not body.asleep:
            return
        stack = [body] # 깨울 물체 목록
        while stack:
            b = stack.pop()
            if not b.asleep: # 이미 깼으면 건너뛰기!
                continue
            b.asleep = False
            b.sleep_timer = 0.0
            self.events["wake"] += 1

            # 같은 아일랜드의 잠든 물체도 깨울 목록에 (chain_wake와 상관없이)
            if self.settings.islands and b.island is not None:
                stack.extend(m for m in b.island.bodies if m.asleep)
            b.island = None
            
            # 깬 물체의 이웃 기록은 지우기
            neighbors = b.sleep_neighbors
            b.sleep_neighbors = set()
            for n in neighbors:
                n.sleep_neighbors.discard(b) # 상대 기록에서도 나를 지우기
                if self.settings.chain_wake and n.asleep:
                    stack.append(n) # 잠든 이웃도 깨울 목록에

    def on_remove(self, body):
        # 물체가 사라질 때, 받치던 이웃이 공중에 뜨지 않도록 깨우기!!
        
        # 1) 사라지는 물체를 이웃 기록과 아일랜드에서 먼저 지운다
        neighbors = body.sleep_neighbors
        body.sleep_neighbors = set()
        for n in neighbors:
            n.sleep_neighbors.discard(body)
        isl = body.island
        body.island = None
        if isl is not None and body in isl.bodies:
            isl.bodies.remove(body)
        
        # 2) 받치던 이웃이 공중에 뜨지 않도록 깨운다
        if self.settings.chain_wake:
            for n in neighbors:
                self.wake(n)
            if isl is not None:
                for m in list(isl.bodies):
                    self.wake(m)


    def on_touch(self, sleeper, awake):
        s = self.settings
        
        # 단순 방식: 누가 닿기만 하면 무조건 깨우기
        if s.wake_rule == "any":
            self.wake(sleeper)
            return True
        # 상대가 유의미하게 움직이고 있을 때만 깨우기
        v = awake.velocity
        if v.x * v.x + v.y * v.y > s.wake_speed * s.wake_speed:
            self.wake(sleeper)
            return True
        return False

    def update(self, bodies, dt=config.DT, contacts=()):
        if not self.enabled():
            return

        s = self.settings
        need = s.time_to_sleep if s.use_timer else 0.0 # 타이머가 꺼졌을 땐 0초 -> 즉시, NAIVE

        # "누가 누구와 닿아 있나" 표 만들기!
        # ex: {상자A: [상자B, 상자C], 상자B: [상자A], ...}

        touching = {}
        for c in contacts:
            if c.a.is_static or c.b.is_static:
                continue
            touching.setdefault(c.a, []).append(c.b)
            touching.setdefault(c.b, []).append(c.a)
        
        # island 끄면: 똑같이 한 물체씩 판단

        if not s.islands:
            for b in bodies:
                if b.is_static or b.asleep:
                    continue
                if self.is_slow(b):
                    b.sleep_timer += dt
                    if b.sleep_timer >= need:
                        self.put_to_sleep(b, touching.get(b, ()))
                else:
                    b.sleep_timer = 0.0
            return

        # 1) 타이머만 먼저 갱신 (잠들지는 않음) -> 깨어있는 물체 고르기!
        awake = [b for b in bodies if not b.is_static and not b.asleep]
        for b in awake:
            if self.is_slow(b):
                b.sleep_timer += dt
            else:
                b.sleep_timer = 0.0

        # 2) 닿아 있는 물체끼리 묶기 (touching에 정적 물체가 없으니 바닥으로는 안 묶임)
        uf = UnionFind()
        for b in awake:
            uf.find(b)                      # 아무와도 안 닿은 물체도 혼자 한 묶음
        for a, others in touching.items():
            for o in others:
                uf.union(a, o)
        groups = {}                         # 대표 → 깨어 있는 구성원 목록
        for b in awake:
            groups.setdefault(uf.find(b), []).append(b)

        # 3) 묶음 전체가 충분히 느렸을 때만 함께 잠듦
        for members in groups.values():
            if not all(m.sleep_timer >= need for m in members):
                continue                    # 하나라도 아직이면 모두 기다림
            
            # 이미 잠든 아일랜드에 닿아 있으면 그 아일랜드와 하나로 합침
            old = set()
            for m in members:
                for o in touching.get(m, ()):
                    if o.asleep and o.island is not None:
                        old.add(o.island)
            island = Island(list(members))
            for isl in old:
                island.bodies.extend(isl.bodies)
                for m in isl.bodies:
                    m.island = island       # 옛 아일랜드 구성원도 새 아일랜드를 가리키게
            for m in members:
                self.put_to_sleep(m, touching.get(m, ()), island)