"""High-level strategy code"""

# !v DEBUG ONLY
import math  # type: ignore  # noqa: F401
from time import time  # type: ignore  # noqa: F401
from typing import Optional

from raspberry.bridge import const
from raspberry.bridge.auxiliary import rbt  # type: ignore  # noqa: F401
from raspberry.bridge.auxiliary import aux, fld
from raspberry.bridge.const import State as GameStates
from raspberry.bridge.router.actions import (  # type: ignore  # noqa: F401
    Action,
    Actions,
    KickActions,
)

from bridge.strategy import gamestates, gamestates_gk



class Strategy:
    """Main class of strategy"""

    def __init__(self,) -> None:
        self.we_active = False
        self.last_ball_pos = [aux.Point(0,0) for i in range(11)]
        self.gk_game_state = 0
        self.global_state = 0
        self.num_allies = [4, 2, 6]
        self.num_enemies = [3, 5, 7]
        
    def run(self,field: fld.Field, actions: list[Optional[Action]]) -> None:
        
        """
        ONE ITERATION of strategy
        NOTE: robots will not start acting until this function returns an array of actions,
              if an action is overwritten during the process, only the last one will be executed)

        Examples of getting coordinates:
        - field.allies[8].get_pos(): aux.Point -   coordinates  of the 8th  robot from the allies
        - field.enemies[14].get_angle(): float - rotation angle of the 14th robot from the opponents

        - field.ally_goal.center: Point - center of the ally goal
        - field.enemy_goal.hull: list[Point] - polygon around the enemy goal area


        Examples of robot control:
        - actions[2] = Actions.GoToPoint(aux.Point(1000, 500), math.pi / 2)
                The robot number 2 will go to the point (1000, 500), looking in the direction π/2 (up, along the OY axis)

        - actions[3] = Actions.Kick(field.enemy_goal.center)
                The robot number 3 will hit the ball to 'field.enemy_goal.center' (to the center of the enemy goal)

        - actions[9] = Actions.BallGrab(0.0)
                The robot number 9 grabs the ball at an angle of 0.0 (it looks to the right, along the OX axis)
        """
        actions[0] = KickActions.goal_kick()
        print(field.ally_goal.frw)