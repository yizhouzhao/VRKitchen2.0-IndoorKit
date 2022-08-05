import os
import json
import math

import pxr 
import omni
import carb

from .base_checker import BaseChecker

class OrientChecker(BaseChecker):
    def __init__(self, task_type, task_id, robot_id, mission_id, annotator="Steven", tolerance = 15) -> None:
        super().__init__(task_type, task_id, robot_id, mission_id, annotator)
        
        
        assert len(self.current_mission) > 0, "mission needs to exist"

        # angle tolerance
        self.tolerance = tolerance
        
        # get target goal condition
        cond = self.current_mission["goal"]["condition"]
        target_obj = cond["target"]
        self.target_prim_path =  "/World/game/" + target_obj
        self.target_delta_y = cond["target_value"]

        from omni.isaac.core.prims import XFormPrim, RigidPrim
        self.targetRigid = RigidPrim(self.target_prim_path)
        self.previous_pos = None
        self.vel = None

        self.record_init_cond()
        print(f"Initialize {self.task_type}, task {self.task_id}, mission {self.mission_id}")
        carb.log_warn("Task desc:" + self.current_mission["goal"]["description"])

        #override interval:
        self.checking_interval = 1

  
    def record_init_cond(self):
        self.target_prim = self.stage.GetPrimAtPath(self.target_prim_path)
        if not self.target_prim:
            raise Exception(f"Target prim must exist at path {self.target_prim_path}")
        print("target_prim: ", self.target_prim.GetPath())
        # # get transform
        # mat = omni.usd.utils.get_world_transform_matrix(self.target_prim) 
        # # self.target_prim_init_y = mat.ExtractTranslation()[1] # extract y axis
        # self.init_orient = mat.ExtractRotation().GetQuat()
    
    def get_prim_y_angle(self):
        """
        Get prim at angle difference from [0,1,0]
        """
        
        mat = omni.usd.utils.get_world_transform_matrix(self.target_prim) 

        y = mat.GetColumn(1)
        # print("mat", mat, "\n column", y)
        cos_angle = y[1] / math.sqrt(y[0]**2 + y[1]**2 + y[2]**2)
        return math.degrees(math.acos(cos_angle))

    def get_diff(self):
        delta_angle = self.get_prim_y_angle() - self.target_delta_y

        return delta_angle

    def start_checking(self):
        # success condition
        self.total_step += 1
        if self.total_step % self.checking_interval == 0:
            delta_angle = abs(self.get_prim_y_angle() - self.target_delta_y)

            pos, rot = self.targetRigid.get_world_pose()
            pos = pos[1]
            
            if self.previous_pos is not None:
                self.vel  = abs(pos - self.previous_pos)
            
            
            if self.total_step % self.print_every == 0:
                print("delta_angle", delta_angle)

            if delta_angle < self.tolerance and self.vel is not None and self.vel < 0.1:
                self.success_steps += self.checking_interval
                self._on_success_hold()
            else:
                self._on_not_success()
            self.previous_pos = pos
            super().start_checking()
         
        




        

    
