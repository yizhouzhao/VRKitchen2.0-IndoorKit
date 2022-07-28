import omni
from omni.isaac.core import World
from omni.isaac.core.physics_context import PhysicsContext
from omni.isaac.franka import Franka
import carb
from omni.isaac.core.utils.stage import (
    get_current_stage,
)
from pxr import UsdGeom, UsdLux, Gf, Vt, UsdPhysics, PhysxSchema, Usd, UsdShade, Sdf
import os
from omni.kitchen.asset.param import ROBOT_PATH

class FrankaSetup():
    def __init__(self, position, rotation, parent_path = None) -> None:
 
        robot_path = parent_path + "/franka" if parent_path else "/franka"
        self._frankabot = Franka(
                prim_path = robot_path, name = "my_franka",
                usd_path = os.path.join(ROBOT_PATH, "franka/franka.usd"),
                orientation = rotation,
                position = position,
                end_effector_prim_name = 'panda_rightfinger',
                gripper_dof_names = ["panda_finger_joint1", "panda_finger_joint2"],
            )

        self.stage = get_current_stage()
        
        
    def get_franka(self):
        return self._frankabot