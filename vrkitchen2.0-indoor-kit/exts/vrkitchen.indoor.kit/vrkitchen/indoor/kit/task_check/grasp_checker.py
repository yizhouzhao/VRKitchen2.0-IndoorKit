import os
import json

import pxr 
import omni
import carb

from .base_checker import BaseChecker

class GraspChecker(BaseChecker):
    def __init__(self, task_type, task_id, robot_id, mission_id, annotator="Steven", tolerance = 10) -> None:
        super().__init__(task_type, task_id, robot_id, mission_id, annotator)
        
        assert len(self.current_mission) > 0, "mission needs to exist"
        self.tolerance = tolerance

        # get target goal condition
        self.cond = self.current_mission["goal"]["condition"]
        target_obj = self.cond["target"]
        self.target_prim_path =  "/World/game/" + target_obj
        self.target_delta_y = self.cond["target_value"]

        self.is_init = True
        print(f"Initialize {self.task_type}, task {self.task_id}, mission {self.mission_id}")
        carb.log_warn("Task desc:" + self.current_mission["goal"]["description"])
        from omni.isaac.core.prims import XFormPrim, RigidPrim
            # import numpy as np
        self.targetRigid = RigidPrim(self.target_prim_path)
        self.previous_pos = None
        self.vel = None

        #override interval:
        self.checking_interval = 1


    def record_init_cond(self):
        self.target_prim = self.stage.GetPrimAtPath(self.target_prim_path)
        if not self.target_prim:
            raise Exception(f"Target prim must exist at path {self.target_prim_path}")
        
        # get transform
        mat = omni.usd.utils.get_world_transform_matrix(self.target_prim) 
        self.target_prim_init_y = mat.ExtractTranslation()[1] # extract y axis
        # print("init_y", self.target_prim_init_y)
        # from omni.isaac.core.prims.xform_prim import XFormPrim
        # xform  = XFormPrim(self.target_prim_path)
        # self.target_prim_init_y =  xform.get_world_pose()[0][1] * xform.get_world_scale()[1]
        # self.target_prim_init_y =  XFormPrim(self.target_prim_path).get_world_pose()[0][1]

    def get_diff(self):
        mat = omni.usd.utils.get_world_transform_matrix(self.target_prim) 
        target_prim_current_y = mat.ExtractTranslation()[1]
        need_delta_y = target_prim_current_y - (self.target_delta_y + self.target_prim_init_y)

        return need_delta_y
    
    def start_checking(self):
        if self.is_init:
            self.record_init_cond()
            self.is_init = False
            return 
        self.total_step += 1
        if self.total_step % self.checking_interval == 0:
            mat = omni.usd.utils.get_world_transform_matrix(self.target_prim) 
            target_prim_current_y = mat.ExtractTranslation()[1]
            
            # self.target_prim_path 
            
            # from omni.isaac.core.prims.xform_prim import XFormPrim
            # xform  = XFormPrim(self.target_prim_path)
            # print("Scale: ", xform.get_world_scale())
            # target_prim_current_y =  xform.get_world_pose()[0][1] * xform.get_world_scale()[1]
            # scale = get_world_scale()

            
            
            # target.initialize()
            pos, rot = self.targetRigid.get_world_pose()
            pos = pos[1]
            
            if self.previous_pos is not None:
                self.vel  = abs(pos - self.previous_pos)
            
            target_height = (self.target_delta_y + self.target_prim_init_y)
            need_delta_y = abs(target_prim_current_y - target_height)
            if self.total_step % self.print_every == 0:
                print("target height %s current height %s" %(target_height, target_prim_current_y))

            # success condition
            if  need_delta_y < self.tolerance and self.vel is not None and self.vel < 0.1 :
                # self.success = True
                # self._on_success()
                self.success_steps += self.checking_interval
                self._on_success_hold()
            else:
                # self.success = False
                self._on_not_success()
            self.previous_pos = pos
            super().start_checking()
        

        

    
