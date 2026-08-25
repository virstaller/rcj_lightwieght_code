"""
Class with robot actions
"""

from time import time


from raspberry.bridge import const
from raspberry.bridge.auxiliary import tau
from raspberry.bridge.auxiliary import aux
from raspberry.bridge.const import State as GameStates
from raspberry.bridge.router.actions.action import Action, ActionDomain, ActionValues
from raspberry.bridge.router.path_generation.path_generation import (
    avoid_goal_zone,
    calc_passthrough_point,
    correct_target_pos,
)

from .dumb_actions import DumbActions
import math

class Actions:
    """Class with all user-available actions (except kicks)"""

    class Unused(Action):
        is_used = False

    class Stop(Action):
        """Stop the robot"""

        def behavior(self, domain: ActionDomain, current_action: ActionValues) -> None:
            """Behavior"""
            current_action.vel = aux.Point(0, 0)
            current_action.angle = 0.0
            current_action.beep = 1

    class GoToPointIgnore(Action):
        """Go to point ignore obstacles"""

        def __init__(
            self,
            target_pos: aux.Point,
            target_angle: float,
            ball_catching: bool = False,
            target_vel: aux.Point = aux.Point(0, 0),
        ) -> None:
            self.target_pos = target_pos
            self.target_angle = target_angle
            self.ball_catching = ball_catching
            self.target_vel = target_vel

            self.use_dribbler = False

        def behavior(self, domain: ActionDomain, current_action: ActionValues) -> None:
            cur_robot = domain.robot
            vec_err = self.target_pos - cur_robot.get_pos()
            cur_vel = cur_robot.get_vel()
            now = time()

            if self.ball_catching:
                cur_robot.pos_reg_x.select_mode(tau.Mode.CATCH)
                cur_robot.pos_reg_y.select_mode(tau.Mode.CATCH)
            else:
                cur_robot.pos_reg_x.select_mode(tau.Mode.NORMAL)
                cur_robot.pos_reg_y.select_mode(tau.Mode.NORMAL)

            dist_err = vec_err.mag()
            u_x = cur_robot.pos_reg_x.process(vec_err.x, -cur_vel.x, total_dist=dist_err)
            u_y = cur_robot.pos_reg_y.process(vec_err.y, -cur_vel.y, total_dist=dist_err)
            current_action.vel = aux.Point(u_x, u_y)

            if not (self.ball_catching and domain.robot.r_id == domain.field.gk_id):
                cur_vel_abs = current_action.vel  # aux.rotate(current_action.vel, cur_robot.get_angle())
                prev_vel_abs = (
                    cur_robot.prev_sended_vel
                )  # aux.rotate(cur_robot.prev_sended_vel, -cur_robot.prev_sended_angle)
                if (cur_vel_abs - prev_vel_abs).mag() / (
                    now - cur_robot.prev_sended_time
                ) > const.MAX_ACCELERATION and cur_vel_abs.mag() > prev_vel_abs.mag():
                    # domain.field.router_image.draw_circle(aux.Point(0, 1000), size_in_mms=200)
                    current_action.vel = prev_vel_abs + (cur_vel_abs - prev_vel_abs).unity() * const.MAX_ACCELERATION * (
                        now - cur_robot.prev_sended_time
                    )

            # current_action.vel = aux.Point(0,500)
            cur_robot.prev_sended_vel = current_action.vel
            cur_robot.prev_sended_angle = cur_robot.get_angle()
            cur_robot.prev_sended_time = now
            current_action.angle = self.target_angle #######################

            if self.use_dribbler:
                current_action.dribbler_speed = 15
            # print('hp', aux.dist(self.target_vel, aux.Point(0, 0)))
            DumbActions.AddFinalVelocityAction(self.target_pos, self.target_vel).process(domain, current_action)

    class GoToPoint(Action):
        """Go to point and avoid obstacles"""

        def __init__(
            self,
            target_pos: aux.Point,
            target_angle: float,
            *,
            ball_catch: bool = False,
            ignore_ball: bool = False,
            target_vel: aux.Point = aux.Point(0, 0),
            ignore_robots: dict[const.Color, list[int]] = {},
        ) -> None:
            self.target_pos = target_pos
            self.target_angle = target_angle
            self.ball_catch = ball_catch
            self.ignore_ball = ignore_ball
            self.target_vel = target_vel
            self.ignore_robots = ignore_robots

        def use_behavior_of(self, domain: ActionDomain, current_action: ActionValues) -> list["Action"]:
            avoid_ball = (
                domain.game_state in [GameStates.STOP, GameStates.PREPARE_KICKOFF]
                or (domain.game_state in [GameStates.FREE_KICK, GameStates.KICKOFF] and not domain.we_active)
            )
            self.target_pos, self.target_vel = correct_target_pos(
                domain.field, domain.robot, self.target_pos, self.target_vel, avoid_ball
            )

            angle0 = self.target_angle
            next_point = self.target_pos

            if domain.robot.r_id != domain.field.gk_id:
                avoid_point = avoid_goal_zone(domain.field, domain.robot, next_point)
                if avoid_point is not None:
                    vel = (avoid_point - domain.robot.get_pos()).unity() * 250
                    domain.field.router_image.draw_circle(avoid_point, size_in_mms=30)

                    return [Actions.GoToPointIgnore(avoid_point, angle0, target_vel=vel)]

            pth_wp = calc_passthrough_point(
                domain,
                next_point,
                avoid_ball=avoid_ball,
                ignore_ball=self.ignore_ball,
                ignore_robots=self.ignore_robots,
            )
            if pth_wp is not None:
                target_speed = min(const.MAX_SPEED, aux.dist(pth_wp, next_point))
                target_vel = (pth_wp - domain.robot.get_pos()).unity() * target_speed
                return [Actions.GoToPointIgnore(pth_wp, angle0, target_vel=target_vel)]

            if next_point != self.target_pos:
                target_speed = min(const.MAX_SPEED * 0.7, aux.dist(self.target_pos, next_point))
                target_vel = (next_point - domain.robot.get_pos()).unity() * target_speed
                return [Actions.GoToPointIgnore(next_point, angle0, target_vel=target_vel)]

            return [Actions.GoToPointIgnore(self.target_pos, angle0, self.ball_catch, self.target_vel)]

    class GoToPointFast(Action):
        def __init__(
            self,
            target_pos: aux.Point,
            target_angle: float,
            *,
            ball_catch: bool = False,
            ignore_ball: bool = False,
            target_vel: aux.Point = aux.Point(0, 0),
            ignore_robots: dict[const.Color, list[int]] = {},
        ) -> None:
            self.target_pos = target_pos
            self.target_angle = target_angle
            self.ball_catch = ball_catch
            self.ignore_ball = ignore_ball
            self.target_vel = target_vel
            self.ignore_robots = ignore_robots

        def use_behavior_of(self, domain: ActionDomain, current_action: ActionValues) -> list["Action"]:
            avoid_ball = (
                domain.game_state in [GameStates.STOP, GameStates.PREPARE_KICKOFF]
                or (domain.game_state in [GameStates.FREE_KICK, GameStates.KICKOFF] and not domain.we_active)
            )
            self.target_pos, self.target_vel = correct_target_pos(
                domain.field, domain.robot, self.target_pos, self.target_vel, avoid_ball
            )

            angle0 = self.target_angle
            next_point = self.target_pos

            if domain.robot.r_id != domain.field.gk_id:
                avoid_point = avoid_goal_zone(domain.field, domain.robot, next_point)
                if avoid_point is not None:
                    vel = (avoid_point - domain.robot.get_pos()).unity() * 250
                    domain.field.router_image.draw_circle(avoid_point, size_in_mms=30)

                    return [Actions.GoToPointIgnore(avoid_point, angle0, target_vel=vel)]
            pth_wp = calc_passthrough_point(
                domain,
                next_point,
                avoid_ball=avoid_ball,
                ignore_ball=self.ignore_ball,
                ignore_robots=self.ignore_robots,
            )
            if pth_wp is not None:
                if(aux.dist(pth_wp, next_point) > 0.1):
                    target_speed = min(const.MAX_SPEED, aux.dist(pth_wp, next_point)*3.5)
                    if(aux.dist(pth_wp, next_point) > 100):
                        target_speed += 400
                    if(aux.dist(pth_wp, next_point) > 50):
                        target_speed += 100
                    # print('ho2')
                    # if (aux.dist(pth_wp, next_point) > 0.00001):
                    #     print('yes')
                    # else:
                    #     print('no')
                    target_vel = (pth_wp - domain.robot.get_pos()).unity() * target_speed
                    return [Actions.GoToPointIgnore(pth_wp, angle0, target_vel=target_vel)]

            if next_point != self.target_pos:
                target_speed = min(const.MAX_SPEED, aux.dist(self.target_pos, domain.robot.get_pos())*3.5)
                if(aux.dist(self.target_pos, domain.robot.get_pos()) > 100):
                    target_speed += 400
                    # print('yes1')
                if(aux.dist(self.target_pos, domain.robot.get_pos()) > 50):
                    target_speed += 100
                    # print('yes2')
                # print('hp0',target_speed)
                target_vel = (next_point - domain.robot.get_pos()).unity() * target_speed
                return [Actions.GoToPointIgnore(next_point, angle0, target_vel=target_vel)]
            target_speed = min(const.MAX_SPEED, aux.dist(self.target_pos, domain.robot.get_pos())*3.5)
            if(aux.dist(self.target_pos, domain.robot.get_pos()) > 100):
                target_speed += 400
            if(aux.dist(self.target_pos, domain.robot.get_pos()) > 50):
                target_speed += 100
            # print('hp0',target_speed)
            return [Actions.GoToPointIgnore(self.target_pos, angle0, self.ball_catch, self.target_vel)]

    class GoToPointBallGrab(Action):
        """Go to point and avoid obstacles"""

        def __init__(
            self,
            target_pos: aux.Point,
            target_angle: float,
            target_vel: aux.Point,
            *,
            ball_catch: bool = False,
            ignore_ball: bool = False,
            ignore_robots: dict[const.Color, list[int]] = {},
        ) -> None:
            self.target_pos = target_pos
            self.target_angle = target_angle
            self.ball_catch = ball_catch
            self.ignore_ball = ignore_ball
            self.target_vel = target_vel
            self.ignore_robots = ignore_robots

        def use_behavior_of(self, domain: ActionDomain, current_action: ActionValues) -> list["Action"]:
            start_dist = aux.dist(self.target_vel, aux.Point(0, 0))
            #print("teor2", start_dist)
            avoid_ball = (
                domain.game_state in [GameStates.STOP, GameStates.PREPARE_KICKOFF]
                or (domain.game_state in [GameStates.FREE_KICK, GameStates.KICKOFF] and not domain.we_active)
            )
            self.target_pos, self.target_vel = correct_target_pos(
                domain.field, domain.robot, self.target_pos, self.target_vel, avoid_ball
            )

            angle0 = self.target_angle
            next_point = self.target_pos

            if domain.robot.r_id != domain.field.gk_id:
                avoid_point = avoid_goal_zone(domain.field, domain.robot, next_point)
                if avoid_point is not None:
                    vel = (avoid_point - domain.robot.get_pos()).unity() * 350
                    domain.field.router_image.draw_circle(avoid_point, size_in_mms=30)
                    #print("fact1", aux.dist(vel, aux.Point(0, 0)))
                    return [Actions.GoToPointIgnore(avoid_point, angle0, target_vel=vel)]

            pth_wp = calc_passthrough_point(
                domain,
                next_point,
                avoid_ball=avoid_ball,
                ignore_ball=self.ignore_ball,
                ignore_robots=self.ignore_robots,
            )
            if pth_wp is not None:
                target_vel = (pth_wp - domain.robot.get_pos()).unity() * start_dist
                #print("fact2", aux.dist(target_vel, aux.Point(0, 0)))
                return [Actions.Velocity(target_vel, angle0)]

            if next_point != self.target_pos:
                target_vel = (next_point - domain.robot.get_pos()).unity() * start_dist
                #print("fact3", aux.dist(target_vel, aux.Point(0, 0)))
                return [Actions.Velocity(target_vel, angle0)]

            return [Actions.GoToPointIgnore(self.target_pos, angle0, self.ball_catch, self.target_vel)]

    class GoToPointLevelUp(Action):
        """Go to point and avoid obstacles"""

        def __init__(
            self,
            target_pos: aux.Point,
            target_angle: float,
            *,
            ball_catch: bool = False,
            ignore_ball: bool = False,
            ignore_robots: dict[const.Color, list[int]] = {},
        ) -> None:
            self.target_pos = target_pos
            self.target_angle = target_angle
            self.ball_catch = ball_catch
            self.ignore_ball = ignore_ball
            self.ignore_robots = ignore_robots

        def use_behavior_of(self, domain: ActionDomain, current_action: ActionValues) -> list["Action"]:
            dist = aux.dist(domain.robot.get_pos(), self.target_pos)
            if dist > 250:
                vel = dist*7 + 250
            elif dist > 100:
                vel = dist*4 + 220
            else:
                vel = dist*3
            
            # print(vel)
            vel_point = (-domain.robot.get_pos() + self.target_pos).unity() * vel
            
            return [Actions.GoToPointBallGrab(self.target_pos, self.target_angle, vel_point)]


    class BallGrab(Action):
        """Grab ball in a given direction"""

        def __init__(self, target_angle: float, *, perform_ball_placement: bool = False) -> None:
            self.target_angle = target_angle
            self.perform_ball_placement = perform_ball_placement

        def is_defined(self, domain: ActionDomain) -> bool:
            it = aux.dist(domain.field.ball.get_pos(), domain.robot.get_pos()) < 1.3*const.ROBOT_R
            return it

        def behavior(self, domain: ActionDomain, current_action: ActionValues) -> None:

            pass

        def use_behavior_of(self, domain: ActionDomain, current_action: ActionValues) -> list["Action"]:
            ignore_ball: bool = False  # условие игнорирования мяча
            p_on_c = - aux.rotate(aux.Point(const.GO_TO_BALL_WITH_ANGLE, 0), -self.target_angle) + domain.field.ball.get_pos()
            domain.field.path_image.draw_circle(p_on_c, (201, 255, 33), 0)
            if(aux.get_angle_between_points(domain.robot.get_pos(), domain.field.ball.get_pos(), p_on_c) > 0):
                rotate = 1
            else:
                rotate = -1
            #print(rotate)
            #domain.field.path_image.draw_circle(domain.field.ball.get_pos(), size_in_mms=const.GO_TO_BALL_WITH_ANGLE + 1)
            if(aux.dist(domain.robot.get_pos(), domain.field.ball.get_pos()) > const.GO_TO_BALL_WITH_ANGLE + 2):
                can = aux.line_circle_intersect(domain.robot.get_pos(), p_on_c, domain.field.ball.get_pos(), const.GO_TO_BALL_WITH_ANGLE - 1, "S")
                if(can):
                    tangets = aux.get_tangent_points(domain.field.ball.get_pos(), domain.robot.get_pos(), const.GO_TO_BALL_WITH_ANGLE)
                    if(len(tangets) == 1):
                        next_position = tangets[0]
                        #domain.field.path_image.draw_circle(tangets[0], (255, 0, 255), 10)
                    elif(len(tangets) > 1):
                        ang0 = abs(aux.get_angle_between_points(tangets[0], domain.robot.get_pos(), p_on_c))
                        ang1 = abs(aux.get_angle_between_points(tangets[1], domain.robot.get_pos(), p_on_c))
                        domain.field.path_image.draw_circle(tangets[0], (255, 0, 255), 10)
                        domain.field.path_image.draw_circle(tangets[1], (255, 0, 255), 10)
                        if(ang0 < ang1):
                            next_position = tangets[0]
                            domain.field.path_image.draw_circle(tangets[0], (255, 0, 255), 15)
                            # domain.field.path_image.draw_circle(tangets[0], (255, 0, 255), 10)
                        else:
                            next_position = tangets[1]
                            domain.field.path_image.draw_circle(tangets[1], (255, 0, 255), 15)
                            # domain.field.path_image.draw_circle(tangets[1], (255, 0, 255), 10)

                        #domain.field.path_image.draw_circle(next_position, (255, 255, 0), 15)                    
                    else:
                        next_position = p_on_c
                else:
                    next_position = p_on_c
                vel = min(1500, (aux.dist(domain.robot.get_pos(), next_position)*2 + 400))
                vel_point = (next_position - domain.robot.get_pos()).unity()*vel
                # print("teor", aux.dist(aux.Point(0, 0), vel_point))
                actions: list[Action] = [
                    Actions.GoToPointBallGrab(next_position,
                        (domain.field.ball.get_pos() - p_on_c).arg(),
                        vel_point
                        ),
                        Actions.SetDribblerSpeed(10)
                ]
                
            else:
                p_on_c = - aux.rotate(aux.Point(const.GO_TO_BALL_WITH_ANGLE, 0), -self.target_angle) + domain.field.ball.get_pos()
                delta_ang = aux.wind_down_angle(aux.get_angle_between_points(domain.robot.get_pos(), domain.field.ball.get_pos(), p_on_c))
                if(abs(delta_ang) < 0.3):
                    next_position = domain.field.ball.get_pos()
                else:
                    v = abs(aux.get_angle_between_points(domain.robot.get_pos(), domain.field.ball.get_pos(), p_on_c))/1.2 + 0.04
                    next_position = aux.rotate((domain.robot.get_pos() - domain.field.ball.get_pos()).unity()*const.GO_TO_BALL_WITH_ANGLE, 0.05*rotate) + domain.field.ball.get_pos()
                vel = min(1500, abs(delta_ang)*40 + 40)
                #vel = 0
                
                vel_point = (next_position - domain.robot.get_pos()).unity()*vel
                domain.field.path_image.draw_line(domain.robot.get_pos(), domain.robot.get_pos() + vel_point*2, (255, 0, 0), 20)
                
                actions: list[Action] = [
                    Actions.Velocity(vel_point,
                        (domain.field.ball.get_pos() - p_on_c).arg()
                    ),
                    Actions.SetDribblerSpeed(10)
                ]
            domain.field.path_image.draw_circle(domain.robot.get_pos(), (255, 0, 0), 5)
            #domain.field.path_image.draw_circle(next_position, (255, 0, 0), 15)
            return actions
        

    class Velocity(Action):
        """Move robot with velocity and angle_speed"""

        def __init__(self, velocity: aux.Point, angle: float, control_angle_by_speed: bool = False) -> None:
            self.velocity = velocity
            self.angle = angle  # angle to turn / angle speed

            self.control_angle_by_speed = control_angle_by_speed

        def behavior(self, domain: ActionDomain, current_action: ActionValues) -> None:

            current_action.vel = self.velocity
            current_action.angle = self.angle

            if self.control_angle_by_speed:
                current_action.beep = 1

    class SetDribblerSpeed(Action):
        def __init__(self, speed: int = 15):
            self.speed = speed

        def behavior(self, domain: ActionDomain, current_action: ActionValues) -> None:
            current_action.dribbler_speed = self.speed