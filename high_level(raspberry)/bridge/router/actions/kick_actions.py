from typing import Optional

from raspberry.bridge import const
from raspberry.bridge.auxiliary import aux

from . import Actions
from .action import Action, ActionDomain, ActionValues
from .dumb_actions import DumbActions


class KickActions:
    """Class with available types of ball kicks"""

    class Kick(Action):
        """Base class"""

        def __init__(
            self,
            target_pos: aux.Point,
            voltage: int = 15,
            is_pass: bool = False,
            is_upper: bool = False,
        ) -> None:
            self.target_pos = target_pos
            self.voltage = voltage  # ignore if is_pass
            self.is_upper = is_upper

            if self.voltage > const.VOLTAGE_SHOOT:
                self.voltage = const.VOLTAGE_SHOOT

            if self.is_upper:
                self.voltage = const.VOLTAGE_UP

            self.pass_pos: Optional[aux.Point] = None
            if is_pass:
                self.pass_pos = self.target_pos

    class Straight(Kick):
        """Grab the ball and kick it straight"""

        def __init__(
            self,
            target_pos: aux.Point,
            voltage: int = 15,
            is_pass: bool = False,
            is_upper: bool = False,
            *,
            perform_ball_placement: bool = False,
        ):
            super().__init__(target_pos, voltage, is_pass, is_upper)

            self.perform_ball_placement = perform_ball_placement

        def use_behavior_of(self, domain: ActionDomain, current_action: ActionValues) -> list["Action"]:

            kick_angle = -aux.angle_to_point(domain.field.ball.get_pos(), self.target_pos)

            actions = [
                Actions.BallGrab(kick_angle, perform_ball_placement=self.perform_ball_placement),
                DumbActions.ShootAction(self.target_pos, self.is_upper),
                DumbActions.ControlVoltageAction(self.voltage, self.pass_pos, self.is_upper),
            ]

            return actions

    class give_pass(Action):
        def __init__(
                            self,
                            target_pos_robot: aux.Point,
                            kick_target: aux.Point,
                            is_upper: bool = False,
                        ):
                    # super().__init__(target_pos_robot, kick_target, is_upper)
                    self.kick_target = kick_target
                    self.target_pos_robot = target_pos_robot
                    self._arrived = False

        def use_behavior_of(self, domain: ActionDomain, current_action: ActionValues) -> list[Action]:
            dist = aux.dist(self.target_pos_robot, domain.robot.get_pos())
            if self._arrived and dist > 150:
                  self._arrived = False
            elif not self._arrived and dist < 80:
                  self._arrived = True
            # print("avav")
            if not domain.field.is_ball_in(domain.robot) and self._arrived:
                actions: list[Action] = [KickActions.Straight(self.kick_target, is_pass=False)]
            elif not self._arrived and domain.field.is_ball_in(domain.robot):
                actions: list[Action] = [Actions.GoToPoint(self.target_pos_robot, aux.angle_to_point(domain.robot.get_pos(), self.kick_target)),Actions.SetDribblerSpeed(10)]
                print("avav")
            # elif self._arrived and domain.field.is_ball_in(do)
            else:
                actions: list[Action] = [Actions.BallGrab(aux.angle_to_point(domain.robot.get_pos(), domain.field.ball.get_pos()))]#[KickActions.Straight(self.kick_target, is_pass = False)]
            print(self._arrived, domain.field.is_ball_in(domain.robot))
            return actions
        
    class take_pass(Kick):
        def __init__(
                    self,
                    ball_pos: aux.Point,
                    target: aux.Point,
                    taker_id: int,
                    is_upper: bool = False,
                ):
            super().__init__(ball_pos, is_upper)
            self.ball_pos = ball_pos
            self.taker_id = taker_id
            self.target_pos = target
        def use_behavior_of(self, domain: ActionDomain, current_action: ActionValues) -> list[Action]:
            robot_pos = domain.field.allies[self.taker_id].get_pos()
            ball_pos = domain.field.ball.get_pos()
            ball_vel = domain.field.ball.get_vel()
            ang = aux.angle_to_point(robot_pos, ball_pos)
            pass_started = ball_vel.mag() > 1000
            
            if not pass_started:
                return [Actions.GoToPoint(self.target_pos, ang)]
            
            intercept_point = aux.closest_point_on_line(self.ball_pos, ball_pos, robot_pos, "L")
            
            if aux.dist(robot_pos, ball_pos) < 300:
                return [Actions.GoToPoint(intercept_point, ang)]
            if(aux.dist(domain.field.allies[self.taker_id].get_pos(), domain.field.ball.get_pos()) < 2000):
                return [Actions.GoToPoint(aux.closest_point_on_line(self.ball_pos, domain.field.ball.get_pos(), domain.field.allies[self.taker_id].get_pos(), "L"), aux.angle_to_point(domain.field.allies[self.taker_id].get_pos(), domain.field.ball.get_pos())), Actions.SetDribblerSpeed(5)]#, Actions.BallGrab(aux.angle_to_point(domain.field.allies[self.taker_id].get_pos(), domain.field.ball.get_pos()))]
            else:
                return [Actions.GoToPoint(self.target_pos, aux.angle_to_point(domain.field.allies[self.taker_id].get_pos(), domain.field.ball.get_pos()), ball_catch=True)]

# class take_pass(Kick):
# 	def __init__(
# 		self,
# 		ball_pos: aux.Point,
# 		target: aux.Point,
# 		taker_id: int,
# 		is_upper: bool = False,
# 	):
# 		super().__init__(target, is_upper=is_upper)
# 		self.ball_pos = ball_pos
# 		self.taker_id = taker_id
# 		self.target_pos = target

# 	def use_behavior_of(self, domain: ActionDomain, current_action: ActionValues) -> list[Action]:
# 		robot_pos = domain.field.allies[self.taker_id].get_pos()
# 		ball_pos = domain.field.ball.get_pos()
# 		ball_vel = domain.field.ball.get_vel()
# 		ang = aux.angle_to_point(robot_pos, ball_pos)

# 		pass_started = ball_vel.mag() > const.PASS_SPEED_THRESHOLD

# 		if not pass_started:
# 			# мяч ещё не летит к нам — стоим и ждём на точке приёма
# 			return [Actions.GoToPoint(self.target_pos, ang)]

# 		# пас пошёл — считаем точку перехвата на линии полёта мяча
# 		intercept_point = aux.closest_point_on_line(self.ball_pos, ball_pos, robot_pos, "L")

# 		if aux.dist(robot_pos, ball_pos) < 400:
# 			return [
# 				Actions.GoToPoint(intercept_point, ang),
# 				Actions.BallGrab(ang),
# 			]

# 		return [Actions.GoToPoint(intercept_point, ang)]
    class goal_kick(Action):
        def use_behavior_of(self, domain: ActionDomain, current_action: ActionValues) -> list[Action]:
                if aux.dist(domain.field.enemies[domain.field.enemy_gk_id].get_pos(), domain.field.enemy_goal.up) < aux.dist(domain.field.enemies[domain.field.enemy_gk_id].get_pos(), domain.field.enemy_goal.down):
                     target = aux.average_point((domain.field.enemy_goal.down, domain.field.enemy_goal.center))
                else:
                     target = aux.average_point((domain.field.enemy_goal.up, domain.field.enemy_goal.center))
                return [KickActions.Straight(target, const.VOLTAGE_SHOOT)]
    
        
    class goal_kick_ally(Action):
        def use_behavior_of(self, domain: ActionDomain, current_action: ActionValues) -> list[Action]:
            if aux.dist(domain.field.allies[domain.field.gk_id].get_pos(), domain.field.ally_goal.up) < aux.dist(domain.field.allies[domain.field.gk_id].get_pos(), domain.field.ally_goal.down):
                    target = (domain.field.ally_goal.down * 7 + domain.field.ally_goal.center)/8
            else:
                    target = (domain.field.ally_goal.up * 7 + domain.field.ally_goal.center)/8
            return [KickActions.Straight(target, const.VOLTAGE_SHOOT)]