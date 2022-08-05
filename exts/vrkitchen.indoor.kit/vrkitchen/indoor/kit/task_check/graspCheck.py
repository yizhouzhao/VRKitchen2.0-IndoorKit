
import gc
import numpy as np

from omni.isaac.core.utils.rotations import euler_angles_to_quat,quat_to_euler_angles, matrix_to_euler_angles
from omni.physx.bindings._physx import SimulationEvent
from omni.physx import get_physx_interface
from random import random
from omni.physx.scripts.physicsUtils import *

from omni.debugdraw import _debugDraw
from omni.isaac.core.utils.rotations import euler_angles_to_quat,quat_to_euler_angles, matrix_to_euler_angles
from omni.isaac.core import World
from omni.isaac.core.prims.xform_prim import XFormPrim
import os
from omni.kitchen.asset.param import ROBOT_PATH
class GraspCheck():
    def __init__(self, controller, articulation_controller, target_name) -> None:
    
        self._world = World.instance()

        self._controller = controller
        self._articulation_controller = articulation_controller
        
        self.target_prim = XFormPrim(target_name)
    
    def start_check(self):

        ee_pos, ee_rot = self._controller._mg.get_end_effector_pose()
        ee_pos *= 100
        target_ee= ee_pos 
        target_ee[1] += 10
        target_rot = euler_angles_to_quat(matrix_to_euler_angles(ee_rot))

        init_prim_pose_pos, _= self.target_prim.get_world_pose()
        for i in range(100):
            actions = self._controller.forward(
                target_end_effector_position=target_ee,
                target_end_effector_orientation=target_rot,
            )

            self._articulation_controller.apply_action(actions)
        
        current_prim_pose, _ = self.target_prim.get_world_pose()

    
    def check_grasped(self):
        self.prim_pose_pos, self.prim_pose_rot = self.target_prim.get_world_pose()
