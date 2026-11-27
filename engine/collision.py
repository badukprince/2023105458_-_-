from engine.vector import Vector2D
from engine.body import Body, Polygon, Rectangle, Circle


def collide(body_1: Body, body_2: Body, include_rotation = True):
    if body_1.shape_type == "Polygon" and body_2.shape_type == "Polygon":
        normal, depth = polygons_collision(body_1, body_2) 
    elif body_1.shape_type == "Circle" and body_2.shape_type == "Circle":
        normal, depth = circles_collision(body_1, body_2)
    elif body_1.shape_type == "Polygon" and body_2.shape_type == "Circle":
        normal, depth = polygon_circle_collision(body_1, body_2)
    elif body_1.shape_type == "Circle" and body_2.shape_type == "Polygon":
        normal, depth = polygon_circle_collision(body_2, body_1)

    if normal is None or depth is None:
        return
    
    if body_1.shape_type == "Polygon" and body_2.shape_type == "Polygon":
        contact_points = polygons_contact_points(body_1, body_2)
    elif body_1.shape_type == "Circle" and body_2.shape_type == "Circle":
        contact_points = circles_contact_points(body_1, body_2)
    elif body_1.shape_type == "Polygon" and body_2.shape_type == "Circle":
        contact_points = polygon_circle_contact_points(body_1, body_2)
    elif body_1.shape_type == "Circle" and body_2.shape_type == "Polygon":
        contact_points = polygon_circle_contact_points(body_2, body_1)
        normal = -normal
        
        
    if include_rotation:
        response_with_rotation(body_1, body_2, normal, depth, contact_points)
    else:
        response(body_1, body_2, normal, depth)

    return contact_points
###############################################################################################################
                        # Practice Code
###############################################################################################################
def response(body_1: Body, body_2: Body, normal_vector: Vector2D, penetration_depth: float):
    normal_vector *= -1

    separate_bodies(body_1, body_2, normal_vector, penetration_depth)
    
    relative_velocity = body_2.velocity - body_1.velocity
    
    penetration_velocity = relative_velocity.dot(normal_vector)

    if penetration_velocity > 0:
        return
    
    r = max(body_1.bounce, body_2.bounce) # average/multiply/min
    j = -(1 + r) * penetration_velocity
    j /= 1 / body_1.mass + 1 / body_2.mass

    impulse = normal_vector * j

    if not body_1.is_static:
        body_1.velocity -= impulse / body_1.mass
    if not body_2.is_static:
        body_2.velocity += impulse / body_2.mass
    



def response_with_rotation(body_1: Body, body_2: Body, normal_vector: Vector2D, penetration_depth: float, contact_point: list[Vector2D]):
    # Reverse the normal vector
    normal_vector *= -1

    # Separate the bodies to prevent overlap (Implement this function if not provided)
    separate_bodies(body_1, body_2, normal_vector, penetration_depth)
    
    # Calculate the contact point (average if multiple points)
    if len(contact_point) == 2: 
        contact_point = (contact_point[0] + contact_point[1]) / 2
    else:
        contact_point = contact_point[0]

    # Vectors from body centers to the contact point
    r_1 = contact_point - body_1.center
    r_2 = contact_point - body_2.center

    # Perpendicular vectors for angular velocity calculation
    r_1_perp = Vector2D(-r_1.y, r_1.x)
    r_2_perp = Vector2D(-r_2.y, r_2.x)

    # Calculate relative velocity at the contact point
    relative_velocity = (body_2.velocity + r_2_perp * body_2.angular_velocity) - (body_1.velocity + r_1_perp * body_1.angular_velocity)
    penetration_velocity = relative_velocity.dot(normal_vector)

    # Skip if bodies are moving away from each other
    if penetration_velocity > 0:
        return

    # Compute the impulse scalar (j)
    r = max(body_1.bounce, body_2.bounce)  # Coefficient of restitution
    j = -(1 + r) * penetration_velocity
    j /= 1 / body_1.mass + 1 / body_2.mass + (r_1_perp.dot(normal_vector) ** 2) / body_1.inertia + (r_2_perp.dot(normal_vector) ** 2) / body_2.inertia

    # Compute the impulse vector
    impulse = normal_vector * j

    # Update linear and angular velocities for body_1 and body_2
    body_1.velocity -= impulse / body_1.mass
    body_1.angular_velocity -= r_1.cross(impulse) / body_1.inertia
    body_2.velocity += impulse / body_2.mass
    body_2.angular_velocity += r_2.cross(impulse) / body_2.inertia



############################################################################################################################################################



def project_circle(center, radius: float, axis: Vector2D):
    direction = axis.normalize()
    direction_and_radius = direction * radius

    p1 = center + direction_and_radius
    p2 = center - direction_and_radius

    min_proj = p1.dot(axis)
    max_proj = p2.dot(axis)

    if min_proj > max_proj:
        min_proj, max_proj = max_proj, min_proj

    return min_proj, max_proj

def polygons_collision(polygon_1: Polygon, polygon_2: Polygon):
    normal = Vector2D(0, 0)
    depth = float('inf')
    
    vertices1 = polygon_1.get_vertices()
    vertices2 = polygon_2.get_vertices()
    
    for i in range(len(vertices1)):
        va = vertices1[i]
        vb = vertices1[(i + 1) % len(vertices1)]
        edge = vb - va
        axis = Vector2D(-edge.y, edge.x).normalize()
        min_a, max_a = project_vertices(polygon_1.get_vertices(), axis)
        min_b, max_b = project_vertices(polygon_2.get_vertices(), axis)

        if min_a >= max_b or min_b >= max_a:
            return None, None

        axis_depth = min(max_b - min_a, max_a - min_b)

        if axis_depth < depth:
            depth = axis_depth
            normal = axis

    # Add axes from polygon_2
    for i in range(len(vertices2)):
        va = vertices2[i]
        vb = vertices2[(i + 1) % len(vertices2)]
        edge = vb - va
        axis = Vector2D(-edge.y, edge.x).normalize()
        min_a, max_a = project_vertices(polygon_1.get_vertices(), axis)
        min_b, max_b = project_vertices(polygon_2.get_vertices(), axis)

        if min_a >= max_b or min_b >= max_a:
            return None, None

        axis_depth = min(max_b - min_a, max_a - min_b)

        if axis_depth < depth:
            depth = axis_depth
            normal = axis

    direction = (polygon_1.center - polygon_2.center).normalize()

    if direction.dot(normal) < 0:
        normal *= -1


    return normal, depth

def polygon_circle_collision(polygon: Polygon, circle: Circle):
    assert polygon.shape_type == "Polygon" and circle.shape_type == "Circle", \
        "Shape types of polygon and circle must be 'Polygon' and 'Circle' respectively."
    
    normal = Vector2D(0, 0)
    penetration_depth = float('inf')
    
    vertices = polygon.get_vertices()
    
    for i in range(len(vertices)):
        va = vertices[i]
        vb = vertices[(i + 1) % len(vertices)]

        edge = vb - va

        axis = Vector2D(-edge.y, edge.x).normalize()
       
        # project circle onto axis
        min_a, max_a = project_vertices(vertices, axis)
        min_b, max_b = project_circle(circle.center, circle.radius, axis)
        
        if max_a <= min_b or max_b <= min_a:
            return None, None
        
        axis_depth = min(max_b - min_a, max_a - min_b)

        if axis_depth < penetration_depth:
            penetration_depth = axis_depth
            normal = axis
    
    cp_index = find_closest_point_on_polygon(circle.center, vertices)
    
    cp = vertices[cp_index]
    
    axis = (cp - circle.center).normalize()

    min_a, max_a = project_circle(circle.center, circle.radius, axis)
    min_b, max_b = project_vertices(vertices, axis)

    if max_a <= min_b or max_b <= min_a:
        return None, None
    
    axis_depth = min(max_b - min_a, max_a - min_b)

    if axis_depth < penetration_depth:
        penetration_depth = axis_depth
        normal = axis


    direction = (polygon.center - circle.center).normalize()
   
    if direction.dot(normal) < 0:
        normal *= -1
        
    return normal, penetration_depth


def project_vertices(vertices: list[Vector2D], axis: Vector2D):
    min_proj = float('inf')
    max_proj = float('-inf')

    for v in vertices:
        proj = v.dot(axis)

        if proj < min_proj:
            min_proj = proj
        if proj > max_proj:
            max_proj = proj

    return min_proj, max_proj

def project_circle(center, radius: float, axis: Vector2D):
    direction = axis.normalize()
    direction_and_radius = direction * radius

    p1 = center + direction_and_radius
    p2 = center - direction_and_radius

    min_proj = p1.dot(axis)
    max_proj = p2.dot(axis)

    if min_proj > max_proj:
        min_proj, max_proj = max_proj, min_proj

    return min_proj, max_proj

def find_closest_point_on_polygon(circle_center: Vector2D, vertices: list[Vector2D]):
    result = -1
    min_distance = float('inf')

    for i, v in enumerate(vertices):
        dist = Vector2D.distance(v, circle_center)

        if dist < min_distance:
            min_distance = dist
            result = i

    return result

def separate_bodies(body_1: Body, body_2: Body, normal, penetration_depth):
    separation_vector = normal * penetration_depth

    # Another Solution
    # percent = 0.2; slope = 0.01
    #separation_vector = max(penetration_depth - slope, 0) / (1 / body_1.mass + 1 / body_2.mass) * percent * normal

    if body_1.is_static:
        body_2.center += separation_vector
    elif body_2.is_static:
        body_1.center -= separation_vector
    else:
        body_1.center -= separation_vector / 2
        body_2.center += separation_vector / 2

    # Another Solution
    # if body_1.is_static == False:
    #     body_1.center -= separation_vector * 1 / body_1.mass
    # if body_2.is_static == False:
    #     body_2.center += separation_vector * 1 / body_2.mass


def point_to_line_segment_projection(point: Vector2D, a: Vector2D, b: Vector2D):
    ab = b - a 
    ap = point - a 
    
    proj = ap.dot(ab)
    d = proj / ab.dot(ab) 

    if d <= 0:
        contact_point = a
    elif d >= 1:
        contact_point = b
    else: 
        contact_point = a + ab * d

    distance = Vector2D.distance(contact_point, point)

    return contact_point, distance


def polygons_contact_points(polygon_1: Polygon, polygon_2: Polygon):
    epsilon = 0.0005
    min_distance = float('inf')
    contact_point_1 = None
    contact_point_2 = None

    for i in range(len(polygon_1.get_vertices())):
        vp = polygon_1.get_vertices()[i]
        for j in range(len(polygon_2.get_vertices())):
            va = polygon_2.get_vertices()[j]
            vb = polygon_2.get_vertices()[(j + 1) % len(polygon_2.get_vertices())]

            cp, distance = point_to_line_segment_projection(vp, va, vb)

            if contact_point_1 is not None and abs(distance - min_distance) < epsilon and not cp.distance_to(contact_point_1) < epsilon:
                contact_point_2 = cp
            elif distance < min_distance:
                min_distance = distance
                contact_point_2 = None
                contact_point_1 = cp

    for i in range(len(polygon_2.get_vertices())):
        vp = polygon_2.get_vertices()[i]
        for j in range(len(polygon_1.get_vertices())):
            va = polygon_1.get_vertices()[j]
            vb = polygon_1.get_vertices()[(j + 1) % len(polygon_1.get_vertices())]

            cp, distance = point_to_line_segment_projection(vp, va, vb)

            if contact_point_1 is not None and abs(distance - min_distance) < epsilon and not cp.distance_to(contact_point_1) < epsilon:
                contact_point_2 = cp
            elif distance < min_distance:
                min_distance = distance
                contact_point_2 = None
                contact_point_1 = cp

    return [cp for cp in [contact_point_1, contact_point_2] if cp is not None]

def polygon_circle_contact_points(polygon: Polygon, circle: Circle):

    min_distance = float('inf')
    vertices = polygon.get_vertices()
    
    for i in range(len(vertices)):
        va = vertices[i]
        vb = vertices[(i + 1) % len(vertices)]

        cp, distance = point_to_line_segment_projection(circle.center, va, vb)

        if distance < min_distance:
            min_distance = distance
            contact_point = cp

        
    return [contact_point]

def circles_contact_points(body_1: Circle, body_2: Circle):
    normal = (body_2.center - body_1.center).normalize()

    contact_point = body_1.center + normal * body_1.radius

    return [contact_point]

def circles_collision(body_1: Circle, body_2: Circle):
    assert body_1.shape_type == "Circle" and body_2.shape_type == "Circle", \
        "Both body_1 and body_2 must be of shape_type 'Circle' for Circle collision."
    
    distance = Vector2D.distance(body_1.center, body_2.center)

    if distance >= body_1.radius + body_2.radius:

        return None, None
    

    normal_vector = (body_1.center - body_2.center).normalize()

    penetration_depth  = body_1.radius + body_2.radius - distance


    return normal_vector, penetration_depth

# colide에서 감지만 따로 떼어낸 함수.

def detect_contact(body_1: Body, body_2: Body, clipping=True):
    t1, t2 = body_1.shape_type, body_2.shape_type
    if t1 == "Polygon" and t2 == "Polygon":
        normal, depth = polygons_collision(body_1, body_2)
    elif t1 == "Circle" and t2 == "Circle":
        normal, depth = circles_collision(body_1, body_2)
    elif t1 == "Polygon" and t2 == "Circle":
        normal, depth = polygon_circle_collision(body_1, body_2)
    else:
        normal, depth = polygon_circle_collision(body_2, body_1)

    if normal is None or depth is None:
        return None

    if t1 == "Polygon" and t2 == "Polygon":
        points = None
        if clipping: # 클리핑 방식: 맞닿은 변끼리 잘라 내서 접촉점을 최대 2개까지 정확하게 구하기
            points = polygons_contact_points_clip(body_1, body_2, -normal)
        if not points:
            points = polygons_contact_points(body_1, body_2)

    elif t1 == "Circle" and t2 == "Circle":
        points = circles_contact_points(body_1, body_2)
    elif t1 == "Polygon" and t2 == "Circle":
        points = polygon_circle_contact_points(body_1, body_2)
    else:
        points = polygon_circle_contact_points(body_2, body_1)
        normal = -normal

    return -normal, depth, points

# normal 방향으로 가장 튀어나온 꼭짓점에 붙은 두 변 중, normal에 더 수직인 변
def best_edge(vertices, normal):
    n = len(vertices)
    i = max(range(n), key=lambda m: vertices[m].dot(normal))
    v = vertices[i]
    v_next = vertices[(i + 1) % n]
    v_prev = vertices[i - 1]
    left = (v - v_next).normalize()
    right = (v - v_prev).normalize()
    if abs(right.dot(normal)) <= abs(left.dot(normal)):
        return v_prev, v
    return v, v_next

# 선분 p1-p2에서 direction·p >= offset 인 부분만 남긴다
def clip(p1, p2, direction, offset):
    #p1이 얼마나 벽에서 떨어져 있지?
    d1 = direction.dot(p1) - offset
    #p2도
    d2 = direction.dot(p2) - offset

    out = [] # 남길 점들

    if d1 >= 0: # 안쪽이면 남기기
        out.append(p1)
    if d2 >= 0:
        out.append(p2)

    if d1 * d2 < 0:  # 경계를 가로지르면 교점 추가
        t = d1 / (d1 - d2)
        out.append(p1 + (p2 - p1) * t)
    return out

# 접촉점 구하기!!
def polygons_contact_points_clip(polygon_1: Polygon, polygon_2: Polygon, normal: Vector2D):
    e1 = best_edge(polygon_1.get_vertices(), normal)  # 1번이 2번을 향하는 변
    e2 = best_edge(polygon_2.get_vertices(), -normal)  # 2번이 1번을 향하는 변
    
    d1 = (e1[1] - e1[0]).normalize()
    d2 = (e2[1] - e2[0]).normalize() # 각 변이 뻗어있는 방향

    # normal에 더 수직인 변이 기준 변(ref), 다른 쪽이 입사 변(inc)
    
    if abs(d1.dot(normal)) <= abs(d2.dot(normal)):
        ref, inc, ref_dir, out_n = e1, e2, d1, normal
    else:
        ref, inc, ref_dir, out_n = e2, e1, d2, -normal

    # 입사 변을 기준 변의 양 끝 사이로 자른다
    pts = clip(inc[0], inc[1], ref_dir, ref_dir.dot(ref[0]))
    if len(pts) < 2:
        return []
    pts = clip(pts[0], pts[1], -ref_dir, -ref_dir.dot(ref[1]))
    if len(pts) < 2:
        return []

    # 기준 변의 바깥 법선 (상대 물체 쪽)
    face_n = Vector2D(-ref_dir.y, ref_dir.x)
    if face_n.dot(out_n) < 0:
        face_n = -face_n

    # 기준 변 안쪽으로 파고든 점만 남긴다
    limit = face_n.dot(ref[0])
    return [p for p in pts if face_n.dot(p) - limit <= 0.0]