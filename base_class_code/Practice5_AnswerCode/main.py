import pygame
import sys
from practice_code.body import Body, Circle, Rectangle, Polygon
from components.scene import Scene


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
PLAYER_SPEED_X = PLAYER_SPEED_Y = 3
FPS = 360
GRAVITY = 9.8
COLORS = {
    "white": (255, 255, 255),
    "red": (255, 0, 0),
    "green": (0, 255, 0),
    "blue": (0, 0, 255),
    "black": (0, 0, 0),
    "orange": (255, 128, 0),
    "cyan": (0, 255, 255),
}


# Initialize Pygame
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("2D Physics Engine")


rectangle_1 = Rectangle(
    x=450,
    y=250,
    width=25,
    height=25,
    name="Player",
    mass=50,
)

rectangle_2 = Rectangle(
    x=200,
    y=250,
    width=50,
    height=50,
    mass=100,
)

circle_1 = Circle(
    x=400,
    y=200,
    radius=10,
    mass=30,
)

circle_2 = Circle(x=WIDTH / 2 + 200, y=300, radius=25, mass=120, is_static=False)


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


Scene = Scene(
    [
        rectangle_1,
        rectangle_2,
        rectangle_3,
        circle_1,
        circle_2,
        polygon_1,
        platform_1,
        platform_2,
    ],
    GRAVITY
)

move_left = False
move_right = False
move_up = False
move_down = False
dt = 1 / FPS  

# Main game loop
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_LEFT:
                move_left = True
            elif event.key == pygame.K_RIGHT:
                move_right = True
            elif event.key == pygame.K_UP:
                move_up = True
            elif event.key == pygame.K_DOWN:
                move_down = True
        elif event.type == pygame.KEYUP:
            if event.key == pygame.K_LEFT:
                move_left = False
            elif event.key == pygame.K_RIGHT:
                move_right = False
            elif event.key == pygame.K_UP:
                move_up = False
            elif event.key == pygame.K_DOWN:
                move_down = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            mouse_x, mouse_y = pygame.mouse.get_pos()

            new_rect = Rectangle(
                x=mouse_x,
                y=HEIGHT - mouse_y,
                width=20,
                height=20
            )
            Scene.add(new_rect)

    if move_left:
        rectangle_1.velocity[0] -= PLAYER_SPEED_X
    if move_right:
        rectangle_1.velocity[0] += PLAYER_SPEED_X
    if move_up:
        rectangle_1.velocity[1] += PLAYER_SPEED_Y
    if move_down:
        rectangle_1.velocity[1] -= PLAYER_SPEED_Y

    Scene.step(dt)
    screen.fill(COLORS["white"])

    for body in Scene.bodies:
        if body.name == "Player":
            color = COLORS["red"]
        else : 
            color = COLORS["black"]

        if body.shape_type == "Polygon":
            
            pygame.draw.polygon(
                screen,
                color,
                [(vertex.x, HEIGHT - vertex.y) for vertex in body.get_vertices()],
            )
        elif body.shape_type == "Circle":
            color = COLORS["cyan"]
            pygame.draw.circle(
                screen, color, (body.center[0], HEIGHT - body.center[1]), body.radius
            )
            
    for point in Scene._contact_points:
        pygame.draw.circle(screen, COLORS["green"], (point.x, HEIGHT - point.y), 4)

    pygame.display.flip()

    pygame.time.Clock().tick(FPS)
