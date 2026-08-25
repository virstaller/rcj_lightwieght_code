import math
from typing import Optional

from raspberry.bridge import const
from raspberry.bridge.auxiliary import fld
from raspberry.bridge.auxiliary import rbt
from raspberry.bridge.auxiliary.entity import Entity
from raspberry.bridge.router.actions.action import ActionDomain
from raspberry.bridge.auxiliary import aux


def calc_passthrough_point(
	domain: ActionDomain,
	target: aux.Point,
	*,
	avoid_ball: bool = False,
	ignore_ball: bool = False,
	ignore_robots: dict[const.Color, list[int]] = {}
) -> Optional[aux.Point]:
	"""
	Рассчитать ближайшую промежуточную путевую точку через граф видимости
	"""

	robot_pos = domain.robot.get_pos()
	obstacles = []

	def add_robots(robots, skip_self: bool = False):
		for robot in robots:
			if skip_self and robot is domain.robot:
				continue
			ignored_ids = ignore_robots.get(robot.color, [])
			if robot.r_id in ignored_ids:
				continue
			obstacles.append((robot.get_pos(), const.ROBOT_R * 3))

	add_robots(domain.field.active_enemies(True))
	add_robots(domain.field.active_allies(True), skip_self=True)

	if avoid_ball and not ignore_ball:
		obstacles.append((domain.field.ball.get_pos(), const.ROBOT_R * 3))

	def point_inside_any(p: aux.Point, skip_idx: int = -1) -> bool:
		for idx, (center, radius) in enumerate(obstacles):
			if idx == skip_idx:
				continue
			if aux.dist(p, center) < radius:
				return True
		return False

	def blocked(p1: aux.Point, p2: aux.Point, skip_idx: int = -1) -> bool:
		for idx, (center, radius) in enumerate(obstacles):
			if idx == skip_idx:
				continue
			if aux.line_circle_intersect(p1, p2, center, radius * 0.999, "S"):
				return True
		return False

	if not blocked(robot_pos, target):
		return target

	nodes = [robot_pos, target]
	node_owner = [-1, -1]

	for idx, (center, radius) in enumerate(obstacles):
		for anchor in (robot_pos, target):
			if aux.dist(anchor, center) <= radius:
				continue
			tg = aux.get_tangent_points(center, anchor, radius)
			for t in tg:
				if point_inside_any(t, skip_idx=idx):
					continue
				nodes.append(t)
				node_owner.append(idx)

	n = len(nodes)
	adj = [[] for _ in range(n)]
	for i in range(n):
		for j in range(i + 1, n):
			skip = node_owner[i] if node_owner[i] == node_owner[j] else -1
			if not blocked(nodes[i], nodes[j], skip):
				d = aux.dist(nodes[i], nodes[j])
				adj[i].append((j, d))
				adj[j].append((i, d))

	dist_arr = [float("inf")] * n
	prev = [-1] * n
	dist_arr[0] = 0.0
	visited = [False] * n
	for _ in range(n):
		u = -1
		best = float("inf")
		for i in range(n):
			if not visited[i] and dist_arr[i] < best:
				best = dist_arr[i]
				u = i
		if u == -1:
			break
		visited[u] = True
		if u == 1:
			break
		for v, w in adj[u]:
			if dist_arr[u] + w < dist_arr[v]:
				dist_arr[v] = dist_arr[u] + w
				prev[v] = u

	if dist_arr[1] == float("inf"):
		return target

	path = [1]
	while path[-1] != 0:
		path.append(prev[path[-1]])
	path.reverse()

	next_point = nodes[path[1]]
	domain.field.path_image.draw_line(robot_pos, next_point, (0, 255, 0), 20)
	for center, radius in obstacles:
		domain.field.path_image.draw_circle(center, (255, 128, 128), radius)

	return next_point




def correct_target_pos(
    field: fld.Field,
    robot: rbt.Robot,
    target: aux.Point,
    target_vel: aux.Point,
    avoid_ball: bool,
) -> tuple[aux.Point, aux.Point]:
    """Correct target position"""
    ball_pos = field.ball.get_pos()
    field_hull = field.big_hull

    ally_hull = field.ally_goal.hull
    ally_big_hull = field.ally_goal.big_hull
    enemy_hull = field.enemy_goal.hull
    enemy_big_hull = field.enemy_goal.big_hull

    #проверка на поле
    
    target.x = max(min(const.GOAL_DX, target.x), -const.GOAL_DX)
    target.y = max(min(const.GOAL_DY, target.y), -const.GOAL_DY)

    #проверка на вратарскую зону

    pd1 = aux.Point(const.ROBOT_R * 1.2, 0)
    pd2 = aux.Point(0, const.ROBOT_R * 1.2)
    

    p_up_left = aux.Point(-2250, 1500)*field.polarity
    p_up_right = aux.Point(2250, 1500)*field.polarity
    p_down_left = aux.Point(-2250, -1500)*field.polarity
    p_down_right = aux.Point(2250, -1500)*field.polarity
    if(robot.r_id != field.gk_id):
        safe_poly = [field.ally_goal.center_down + pd2, field.ally_goal.frw_down + pd1 + pd2, field.ally_goal.frw_up + pd1 - pd2, field.ally_goal.center_up - pd2, p_up_right, p_up_left, field.enemy_goal.center_down - pd2, field.enemy_goal.frw_down - pd1 - pd2, field.enemy_goal.frw_up - pd1 + pd2, field.enemy_goal.center_up + pd2, p_down_left, p_down_right]
        if aux.is_point_inside_poly(target, ally_big_hull) or aux.is_point_inside_poly(target, enemy_big_hull):
            target = aux.nearest_point_on_poly(target, safe_poly)

    #избегание столкновений роботов(сырое)
    id_use_robot = robot.r_id
    for i in range(const.AMOUNT_ROBOTS//2):
        if i != id_use_robot:
            if aux.dist(target, field.allies[i].get_pos()) < const.SAVE_PLACE_ALLIES_ROBOTS:
                if target == field.allies[i].get_pos():  
                    p_d_ball = - field.allies[i].get_pos()
                else:
                    p_d_ball = - field.allies[i].get_pos() + target
                p_skip_ball = p_d_ball.unity() * const.SAVE_PLACE_ALLIES_ROBOTS + field.allies[i].get_pos()
                target = p_skip_ball

    for i in range(const.AMOUNT_ROBOTS//2):
        if aux.dist(target, field.enemies[i].get_pos()) < const.SAVE_PLACE_ENEMIES_ROBOTS:
            if target == field.enemies[i].get_pos():  
                p_d_ball = - field.enemies[i].get_pos()
            else:
                p_d_ball = - field.enemies[i].get_pos() + target
            p_skip_ball = p_d_ball.unity() * const.SAVE_PLACE_ENEMIES_ROBOTS + field.enemies[i].get_pos()
            target = p_skip_ball

    #проверка на близость к мячу
    if(avoid_ball):
        if aux.dist(target, field.ball.get_pos()) < const.KEEP_BALL_DIST:
            if target == field.ball.get_pos():
                p_d_ball = - field.ball.get_pos()
            else:
                p_d_ball = - field.ball.get_pos() + target
            p_skip_ball = p_d_ball.unity() * const.KEEP_BALL_DIST + field.ball.get_pos()
            target = p_skip_ball

    return target, target_vel


def avoid_goal_zone(field: fld.Field, robot: rbt.Robot, next_point: aux.Point)   -> Optional[aux.Point]:
    # избегание заезда в штрафную в процессе езды
    
    ally_hull = field.ally_goal.hull
    ally_big_hull = field.ally_goal.big_hull

    enemy_hull = field.enemy_goal.hull
    enemy_big_hull = field.enemy_goal.big_hull

    

    target = None
    arr_ally_goal_big_hull = field.ally_goal.big_hull
    if(aux.segment_poly_intersect(robot._pos, next_point, ally_hull) != None):
        if((robot._pos).x > -1750):
            if(next_point.y < 0):
                target = arr_ally_goal_big_hull[1]
            else:
                target = arr_ally_goal_big_hull[2]
        elif(next_point.x > -1750):
            if((robot._pos).y < 0):
                target = arr_ally_goal_big_hull[1]
            else:
                target = arr_ally_goal_big_hull[2]
        else:
            if((robot._pos).y > 0):
                target = arr_ally_goal_big_hull[2]
            else:
                target = arr_ally_goal_big_hull[1]


    arr_enemy_goal_big_hull = field.enemy_goal.big_hull
    if(aux.segment_poly_intersect(robot._pos, next_point, enemy_hull) != None):
        if((robot._pos).x < 1750):
            if(next_point.y < 0):
                target = arr_enemy_goal_big_hull[2]
            else:
                target = arr_enemy_goal_big_hull[1]
        elif(next_point.x < 1750):
            if((robot._pos).y < 0):
                target = arr_enemy_goal_big_hull[2]
            else:
                target = arr_enemy_goal_big_hull[1]
        else:
            if((robot._pos).y > 0):
                target = arr_enemy_goal_big_hull[1]
            else:
                target = arr_enemy_goal_big_hull[2]

    # if(target != None):
    #     print('ho')

    return target





#by Serge всё что ниже недописано, это нерабочий вариант маршрутизации
# def recurs(domain: ActionDomain, 
#     target: aux.Point,
#     path_points: list[aux.Point],
#     total_dist: float,
#     dep: int,
#     last_problem: aux.Point(),
#     last_problem_radius: float,
#     *,
#     avoid_ball: bool = False,
#     ignore_ball: bool = False,
#     ignore_robots: dict[const.Color, list[int]] = {}
    
# ) -> list[aux.Point | float]:
#     """
#     Считает путь от текущей точки, до следующего препятствия до цели, в том числе использую текущее препятствие.
#     Возвращает текущее положение, текущее препятствие, суммарно пройденное расстояние на данный момент
#     """
#     current_point = path_points[len(path_points) - 1]
#     if(is_segment_safe(domain, current_point, target, avoid_ball=avoid_ball, ignore_ball=ignore_ball, ignore_robots=ignore_robots)):
#         delta_dist = aux.dist(current_point, target)
#         path_points.append(target)
#         it = path_points.copy()
#         it.append(delta_dist + total_dist)
#         return it
#     if(aux.dist_to_line(current_point, target, last_problem, "S") < last_problem_radius):
#         tangets_to_target = aux.get_tangent_points(last_problem, target, last_problem_radius)
#         tanget_to_target = aux.nearest_point_of_points([tangets_to_target[0], tangets_to_target[1]], current_point)
#         next_point = aux.get_line_intersection(target, tanget_to_target, current_point, aux.rotate(current_point - last_problem, math.pi/2) + current_point)
#         total_dist += aux.dist(next_point, current_point)
#     min_dist_to_target = math.inf
#     new_path_points = path_points

#     if(dep >= 10):
#         it = path_points.copy()
#         it.append(math.inf)
#         return it

#     arr_try_path_points = []
#     arr_new_total_dists = []
    
#     for i in range(const.AMOUNT_ROBOTS//2):
#         if i != domain.robot.r_id:
#             if(aux.line_circle_intersect(target, current_point, domain.field.allies[i].get_pos(), const.SAVE_PLACE_ALLIES_ROBOTS, "S")):
#                 tangets_points = aux.get_tangent_points(domain.field.allies[i].get_pos(), current_point, const.SAVE_PLACE_ALLIES_ROBOTS)
#                 for j in range(len(tangets_points)):
#                     if(is_segment_safe(domain, current_point, tangets_points[j], avoid_ball=avoid_ball, ignore_ball=ignore_ball, ignore_robots=ignore_robots)):
#                         domain.field.path_image.draw_line(current_point, tangets_points[j], (201, 255, 33), 15)
#                         delta_dist = aux.dist(tangets_points[j], current_point)
#                         try_path_points = path_points.copy()
#                         try_path_points.append(tangets_points[j])
#                         arr_try_path_points.append(try_path_points)
#                         arr_new_total_dists.append(total_dist + delta_dist)
#                     #else:
                        
                        

#     for i in range(const.AMOUNT_ROBOTS//2):
#         if(aux.line_circle_intersect(target, current_point, domain.field.enemies[i].get_pos(), const.SAVE_PLACE_ENEMIES_ROBOTS, "S")):
#             tangets_points = aux.get_tangent_points(domain.field.enemies[i].get_pos(), current_point, const.SAVE_PLACE_ENEMIES_ROBOTS)
#             if len(tangets_points) > 0:
#                 if(is_segment_safe(domain, current_point, tangets_points[0], avoid_ball=avoid_ball, ignore_ball=ignore_ball, ignore_robots=ignore_robots)):
#                     domain.field.path_image.draw_line(current_point, tangets_points[0], (201, 255, 33), 15)
#                     delta_dist = aux.dist(tangets_points[0], current_point)
#                     try_path_points = path_points.copy()
#                     try_path_points.append(tangets_points[0])
#                     way = recurs(
#                         domain, 
#                         target, 
#                         try_path_points, 
#                         total_dist + delta_dist, 
#                         avoid_ball=avoid_ball, 
#                         ignore_ball=ignore_ball, 
#                         ignore_robots=ignore_robots
#                     )
#                     try_to_target_dist = way[len(way) - 1]
#                     try_to_target_path = way[0: len(way) - 2]
#                     if(try_to_target_dist < min_dist_to_target):
#                         min_dist_to_target = try_to_target_dist
#                         new_path_points = try_to_target_path

#             if len(tangets_points) > 1:
#                 if(is_segment_safe(domain, domain.field.allies[domain.robot.r_id].get_pos(), tangets_points[1], avoid_ball=avoid_ball, ignore_ball=ignore_ball, ignore_robots=ignore_robots)):
#                     domain.field.path_image.draw_line(domain.robot.get_pos(), tangets_points[1], (201, 255, 33), 15)
#                     delta_dist = aux.dist(tangets_points[1], current_point)
#                     try_path_points = path_points.copy()
#                     try_path_points.append(tangets_points[1])
#                     way = recurs(
#                         domain, 
#                         target, 
#                         try_path_points, 
#                         total_dist + delta_dist, 
#                         avoid_ball=avoid_ball, 
#                         ignore_ball=ignore_ball, 
#                         ignore_robots=ignore_robots
#                     )
#                     try_to_target_dist = way[len(way) - 1]
#                     try_to_target_path = way[0: len(way) - 2]
#                     if(try_to_target_dist < min_dist_to_target):
#                         min_dist_to_target = try_to_target_dist
#                         new_path_points = try_to_target_path

#     if(avoid_ball):
#         if(aux.line_circle_intersect(target, current_point, domain.field.ball.get_pos(), const.KEEP_BALL_DIST, "S")):
#             tangets_points = aux.get_tangent_points(domain.field.ball.get_pos(), current_point, const.KEEP_BALL_DIST)
#             if len(tangets_points) > 0:
#                 if(is_segment_safe(domain, current_point, tangets_points[0], avoid_ball=avoid_ball, ignore_ball=ignore_ball, ignore_robots=ignore_robots)):
#                     domain.field.path_image.draw_line(current_point, tangets_points[0], (201, 255, 33), 15)
#                     delta_dist = aux.dist(tangets_points[0], current_point)
#                     try_path_points = path_points.copy()
#                     try_path_points.append(tangets_points[0])
#                     way = recurs(
#                         domain, 
#                         target, 
#                         try_path_points, 
#                         total_dist + delta_dist, 
#                         avoid_ball=avoid_ball, 
#                         ignore_ball=ignore_ball, 
#                         ignore_robots=ignore_robots
#                     )
#                     try_to_target_dist = way[len(way) - 1]
#                     try_to_target_path = way[0: len(way) - 2]
#                     if(try_to_target_dist < min_dist_to_target):
#                         min_dist_to_target = try_to_target_dist
#                         new_path_points = try_to_target_path

#             if len(tangets_points) > 1:
#                 if(is_segment_safe(domain, domain.field.allies[domain.robot.r_id].get_pos(), tangets_points[1], avoid_ball=avoid_ball, ignore_ball=ignore_ball, ignore_robots=ignore_robots)):
#                     domain.field.path_image.draw_line(domain.robot.get_pos(), tangets_points[1], (201, 255, 33), 15)
#                     delta_dist = aux.dist(tangets_points[1], current_point)
#                     try_path_points = path_points.copy()
#                     try_path_points.append(tangets_points[1])
#                     way = recurs(
#                         domain, 
#                         target, 
#                         try_path_points, 
#                         total_dist + delta_dist, 
#                         avoid_ball=avoid_ball, 
#                         ignore_ball=ignore_ball, 
#                         ignore_robots=ignore_robots
#                     )
#                     try_to_target_dist = way[len(way) - 1]
#                     try_to_target_path = way[0: len(way) - 2]
#                     if(try_to_target_dist < min_dist_to_target):
#                         min_dist_to_target = try_to_target_dist
#                         new_path_points = try_to_target_path
            
#     # часть прогона всех новых вариантов
#     for i in range(len(arr_try_path_points)):
#         way = recurs(
#             domain, 
#             target, 
#             arr_try_path_points[i], 
#             arr_new_total_dists[i], 
#             dep + 1,
#             avoid_ball=avoid_ball, 
#             ignore_ball=ignore_ball, 
#             ignore_robots=ignore_robots
#         )
#         try_to_target_dist = way[len(way) - 1]
#         try_to_target_path = way[0: len(way) - 2]
#         if(try_to_target_dist < min_dist_to_target):
#             min_dist_to_target = try_to_target_dist
#             new_path_points = try_to_target_path

#         it = new_path_points.copy()
#         it.append(min_dist_to_target)
#     return it


def is_segment_safe(domain: ActionDomain, 
    segment_start: aux.Point,
    segment_end: aux.Point,
    *,
    avoid_ball: bool = False,
    ignore_ball: bool = False,
    ignore_robots: dict[const.Color, list[int]] = {}
) -> bool:
    it = True
    for i in range(const.AMOUNT_ROBOTS//2):
        if i != domain.robot.r_id:
            if(len(aux.line_circle_intersect(segment_start, segment_end, domain.field.allies[i].get_pos(), const.SAVE_PLACE_ALLIES_ROBOTS - 2, "S")) != 0):
                it = False

    for i in range(const.AMOUNT_ROBOTS//2):
        if(len(aux.line_circle_intersect(segment_start, segment_end, domain.field.enemies[i].get_pos(), const.SAVE_PLACE_ENEMIES_ROBOTS - 2, "S")) != 0):
            it = False

    if(avoid_ball):
        if(len(aux.line_circle_intersect(segment_start, segment_end, domain.field.ball.get_pos(), const.KEEP_BALL_DIST - 2, "S")) != 0):
            it = False

    return it
