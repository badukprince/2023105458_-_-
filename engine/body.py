import math
from engine.vector import Vector2D


class Body():
    def __init__(self, x, y, mass = 1, bounce = 0.5, name = None, is_static = False):
        self.center = Vector2D(x, y)
        self.angle = 0    
        self.name = name
        self.shape_type = None
        self.velocity = Vector2D(0, 0)
        self.angular_velocity = 0
        
        
        self.inertia = None
        self.mass = mass if not is_static else float("inf") #
        self.bounce = bounce 
        self.is_static = is_static
        self.static_friction = 0.5 # 마찰 계수
        self.dynamic_friction = 0.5

        self.asleep = False # True면 적분/충돌 검사 제외, 솔버에서는 벽처럼 취급!
        self.sleep_timer = 0.0 # 느린 상태가 이어진 시간 (초)
        self.sleep_neighbors = set() # 잠들 때 닿아 있던 동적 물체들
        self.island = None # 함께 잠든 묶음

class Rectangle(Body):
    def __init__(self, x, y, width, height, mass = 1, bounce = 0.5, name = None, is_static = False):
        super().__init__(x, y, mass, bounce, name, is_static)
        self.width = width
        self.height = height
        self.shape_type = "Polygon"
        half_width = self.width / 2
        half_height = self.height / 2

        self.local_vertices = [
            Vector2D(-half_width, -half_height),
            Vector2D(half_width, -half_height),
            Vector2D(half_width, half_height),
            Vector2D(-half_width, half_height)
        ]
        #
        self.inertia = (1 / 12) * mass * (width * width + height * height) if not is_static else float("inf")
        
    
    def get_axes(self):
        self.x_axis = Vector2D(math.cos(self.angle), math.sin(self.angle))
        self.y_axis = Vector2D(-math.sin(self.angle), math.cos(self.angle)) 
        
        return [self.x_axis, self.y_axis] 

    def get_vertices(self):
        return [vertex.rotate(self.angle).add(self.center) for vertex in self.local_vertices]

    def rotate(self, angle, in_radians=True):
        if not in_radians:
            angle = math.radians(angle)
        self.angle += angle



class Polygon(Body):
    def __init__(self, x, y, vertices: list[Vector2D, list, tuple], mass=1, bounce=0.5, name=None, is_static=False):
        super().__init__(x, y, mass, bounce, name, is_static)
        centroid = (
            sum(vertex[0] for vertex in vertices) / len(vertices),
            sum(vertex[1] for vertex in vertices) / len(vertices),
        )

        self.local_vertices = [Vector2D(vertex[0] - centroid[0], vertex[1] - centroid[1]) for vertex in vertices]
      
        self.shape_type = "Polygon"
        self.inertia = self.calculate_inertia() if not is_static else float("inf")#
    
    
    def get_vertices(self):
        return [vertex.rotate(self.angle).add(self.center) for vertex in self.local_vertices]
    
    ####
    def calculate_inertia(self):
        area = 0
        center = Vector2D(0, 0)
        mmoi = 0

        prev = len(self.local_vertices) - 1
        for index in range(len(self.local_vertices)):
            a = self.local_vertices[prev]
            b = self.local_vertices[index]

            area_step = a.cross(b) / 2
            center_step = (a + b) / 3
            mmoi_step = area_step * (a.dot(a) + b.dot(b) + a.dot(b)) / 6

            center = (center * area + center_step * area_step) / (area + area_step)
            area += area_step
            mmoi += mmoi_step

            prev = index

        density = self.mass / area
        mmoi *= density
        mmoi -= self.mass * center.dot(center)

        return mmoi

    def rotate(self, angle, in_radians=True):
        if not in_radians:
            angle = math.radians(angle)
        self.angle += angle

class Circle(Body):
    def __init__(self, x, y, radius, mass = 1, bounce = 0.5, name = None, is_static = False):
        super().__init__(x, y, mass, bounce, name, is_static)
        self.radius = radius
        self.shape_type = "Circle"
        self.inertia = (1 / 2) * mass * radius * radius if not is_static else float("inf")

    def rotate(self, angle, in_radians=True):
        if not in_radians:
            angle = math.radians(angle)
        self.angle += angle

