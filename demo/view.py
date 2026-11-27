import os
import pygame
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # 프로젝트 폴더를 import 경로에 (어디서 실행해도 engine을 찾도록)
from engine.body import Body, Rectangle, Polygon
from engine.world import World, PhysicsSettings     
from engine import config       
from engine.sleeping import SleepSettings                     


# References:
# https://2dengine.com/doc/collisions.html
# https://habr.com/en/articles/336908/
# https://code.tutsplus.com/how-to-create-a-custom-2d-physics-engine-the-basics-and-impulse-resolution--gamedev-6331t
# https://habr.com/en/articles/509568/
# https://www.jeffreythompson.org/collision-detection/table_of_contents.php
# https://dyn4j.org/2010/01/sat/
# https://dyn4j.org/2011/11/contact-points-using-clipping/
# https://www.codezealot.org/archives/88/#gjk-support
# https://en.wikipedia.org/wiki/Collision_response#Impulse-based_friction_model
# https://code.tutsplus.com/how-to-create-a-custom-2d-physics-engine-friction-scene-and-jump-table--gamedev-7756t
# https://research.ncl.ac.uk/game/mastersdegree/gametechnologies/physicstutorials/5collisionresponse/Physics%20-%20Collision%20Response.pdf
# https://chrishecker.com/images/e/e7/Gdmphys3.pdf
# https://perso.liris.cnrs.fr/nicolas.pronost/UUCourses/GamePhysics/lectures/lecture%207%20Collision%20Resolution.pdf
# https://code.tutsplus.com/how-to-create-a-custom-2d-physics-engine-oriented-rigid-bodies--gamedev-8032t
# https://stackoverflow.com/questions/31106438/calculate-moment-of-inertia-given-an-arbitrary-convex-2d-polygon

WIDTH, HEIGHT = 800, 600
FPS = 120
COLORS = {
    "white": (255, 255, 255),
    "green": (0, 255, 0),
    "blue": (0, 0, 255),
    "black": (0, 0, 0),
    "orange": (255, 128, 0),
    "gray": (120, 120, 120),
}
BOX = 24          # 클릭으로 만드는 상자 크기
TILT_FLOORS, TILT_SHIFT = 8, 3 # 오른쪽 클릭: 8단, 한 층마다 3 px씩 비켜 쌓은 기운 탑
SLEEP_MODES = [SleepSettings.off, SleepSettings.naive, SleepSettings.improved]

# Initialize Pygame
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("2D Physics Engine")
font = pygame.font.SysFont("malgungothic", 15) # 한글이 나오는 글꼴 (맑은 고딕)


rectangle_2 = Rectangle(
    x=200,
    y=250,
    width=50,
    height=50,
    mass=100,
)

polygon_1 = Polygon(
    x=310,
    y=250,
    vertices=[(0, 0), (50, 0), (50, 25), (0, 50)],
    mass=100,
)

polygon_1.rotate(90, in_radians=False)

rectangle_3 = Rectangle(
    x=WIDTH / 2,
    y=100,
    width=WIDTH / 1.5,
    height=20,
    is_static=True,
    bounce=0.7,
    name="Ground",
)

platform_1 = Rectangle(
    x=600, y=300, width=200, height=20, is_static=True, bounce=0.8
)

platform_1.rotate(30, in_radians=False)

platform_2 = Rectangle(
    x=70, y=200, height=20, width=200, is_static=True, bounce=0.8
)

platform_2.rotate(-60, in_radians=False)


Scene = World(
    [
        rectangle_2,
        rectangle_3,
        polygon_1,
        platform_1,
        platform_2,
    ],
    PhysicsSettings.class_like(),  # 시작은 수업 엔진
    sleep=SleepSettings.off(),     # 휴면 OFF
)

clock = pygame.time.Clock()
dt = config.DT

# Main game loop
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_c:            # C: 수업 엔진 ↔ 안정화 구현
                if Scene.settings == PhysicsSettings.class_like():
                    Scene.settings = PhysicsSettings()
                else:
                    Scene.settings = PhysicsSettings.class_like()
            elif event.key == pygame.K_s:            # S: 휴면 OFF → NAIVE → IMPROVED
                names = [m().mode for m in SLEEP_MODES]
                i = names.index(Scene.sleep.settings.mode)
                new = SLEEP_MODES[(i + 1) % len(SLEEP_MODES)]()
                if new.mode == "OFF":
                    for b in Scene.bodies:           # 끌 때 잠든 물체를 모두 깨움 (안 그러면 그대로 얼어 있음)
                        Scene.sleep.wake(b)
                Scene.sleep.settings = new
            elif event.key == pygame.K_1:            # 1~4: 해결1~4 하나씩 켜기/끄기
                ss = Scene.sleep.settings
                Scene.sleep.settings = ss.but(use_timer=not ss.use_timer)
            elif event.key == pygame.K_2:
                ss = Scene.sleep.settings
                Scene.sleep.settings = ss.but(wake_rule="any" if ss.wake_rule == "moving" else "moving")
            elif event.key == pygame.K_3:
                ss = Scene.sleep.settings
                Scene.sleep.settings = ss.but(chain_wake=not ss.chain_wake)
            elif event.key == pygame.K_4:
                ss = Scene.sleep.settings
                Scene.sleep.settings = ss.but(islands=not ss.islands)
            elif event.key == pygame.K_r:            # R: 움직이는 물체 전부 지우기
                for b in list(Scene.bodies):
                    if not b.is_static:
                        Scene.remove(b)

        elif event.type == pygame.MOUSEBUTTONDOWN:
            mouse_x, mouse_y = pygame.mouse.get_pos()
            x, y = mouse_x, HEIGHT - mouse_y         # 화면 좌표 → 물리 좌표 (y 뒤집기)

            hit = Scene.body_at(x, y)                # [클릭한 곳에 물체가 있나?
            if hit is not None:
                Scene.remove(hit)                    # 있으면 지움 → 받치던 물체를 깨울 기회
            elif event.button == 1:                  # 왼쪽 클릭: 상자 하나
                Scene.add(Rectangle(x=x, y=y, width=BOX, height=BOX))
            elif event.button == 3:                  # 오른쪽 클릭: 8단 기운 탑 (한 층마다 3 px씩 오른쪽으로, 보고서 2.2와 같음)
                for i in range(TILT_FLOORS):
                    Scene.add(Rectangle(x=x + i * TILT_SHIFT, y=y + i * (BOX + 0.5), width=BOX, height=BOX))

    Scene.step(dt)
    for b in list(Scene.bodies): #화면 아래로 떨어진 물체는 지움 (계속 깨어 있으니까)
        if b.center.y < -100:
            Scene.remove(b)
    
    screen.fill(COLORS["white"])

    for body in Scene.bodies:
        if body.asleep:
            color = COLORS["blue"]      
        elif Scene.sleep.enabled() and body.sleep_timer > 0.0:
            color = COLORS["orange"]                 # 느려서 타이머가 쌓이는 중          
        else :
            color = COLORS["black"]

        if body.shape_type == "Polygon":
            points = [(vertex.x, HEIGHT - vertex.y) for vertex in body.get_vertices()]
            pygame.draw.polygon(screen, color, points)
            pygame.draw.polygon(screen, COLORS["white"], points, 1)   # 테두리 (붙은 상자 구분)

    for contact in Scene.contacts: # _contact_points → contacts 안의 points
        for point in contact.points:
            pygame.draw.circle(screen, COLORS["green"], (point.x, HEIGHT - point.y), 4)

    s = Scene.settings  # 현재 설정 글자로 표시
    ss = Scene.sleep.settings
    on = lambda v: "켜짐" if v else "꺼짐"
    engine = "수업 엔진" if s == PhysicsSettings.class_like() else "안정화 구현"
    wake = "닿으면 깨움" if ss.wake_rule == "any" else "움직일 때만"
    dynamic = [b for b in Scene.bodies if not b.is_static]
    lines = [
        f"[C] 엔진: {engine}",
        f"[S] 휴면 모드: {ss.mode}",
        f"[1] 타이머 {on(ss.use_timer)}   [2] 깨우기 규칙 {wake}   "
        f"[3] 연쇄 깨우기 {on(ss.chain_wake)}   [4] 아일랜드 {on(ss.islands)}",
        f"깨어 있는 물체 {Scene.profiler.last.get('awake')}/{len(dynamic)}    "
        f"스텝 {Scene.profiler.mean('step_ms', 60):.2f} ms    "
        f"잠듦 {Scene.sleep.events['sleep']}회 / 깸 {Scene.sleep.events['wake']}회",
        f"왼쪽 클릭: 상자 하나    오른쪽 클릭: 8단 기운 탑    상자 클릭: 삭제    [R] 비우기",
    ]
    for i, line in enumerate(lines):
        screen.blit(font.render(line, True, COLORS["black"]), (10, 10 + 20 * i))

    pygame.display.flip()

    pygame.time.Clock().tick(FPS)
