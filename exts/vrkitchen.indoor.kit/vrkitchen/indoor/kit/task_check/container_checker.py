import os
import json

import pxr 
import omni
import carb

from .base_checker import BaseChecker

from ..param import IS_IN_ISAAC_SIM

class ContainerChecker(BaseChecker):
    def __init__(self, task_type, task_id, robot_id, mission_id, annotator="Steven") -> None:
        super().__init__(task_type, task_id, robot_id, mission_id, annotator)
        
        assert len(self.current_mission) > 0, "mission needs to exist"

        cond = self.current_mission["goal"]["condition"]
        target_obj = cond["target"]
        container_obj = cond["container"]
        self.target_prim_path =  "/World/game/" + target_obj
        self.container_prim_path = "/World/game/" + container_obj
        self.in_or_out = cond["target_value"] == -1
            

        self.record_init_cond()
        print(f"Initialize {self.task_type}, task {self.task_id}, mission {self.mission_id}")
        carb.log_warn("Task desc:" + self.current_mission["goal"]["description"])

    
    def record_init_cond(self):
        self.target_prim = self.stage.GetPrimAtPath(self.target_prim_path)
        if not self.target_prim:
            raise Exception(f"Target prim must exist at path {self.target_prim_path}")
        
        self.container_prim = self.stage.GetPrimAtPath(self.container_prim_path)
        if not self.container_prim:
            raise Exception(f"Container prim must exist at path {self.container_prim_path}")
        
        # scene
        self.stage = omni.usd.get_context().get_stage()

    def contains(self, bbox1, bbox2):
        """
        check one bounding box in another one
        only consider the (x, z) plane, soft-condition on y-axix
        ::params:
            bbox1: target obj bbox
            bbox2: container bbox
        """

        x_in = bbox2[0][0] <= bbox1[0][0] <= bbox1[1][0] <= bbox2[1][0]
        z_in = bbox2[0][2] <= bbox1[0][2] <= bbox1[1][2] <= bbox2[1][2]
        y_in = bbox2[0][1] <= bbox1[0][1] <= bbox2[1][1] <= bbox2[1][1]

        return x_in and z_in and y_in

    def start_checking(self):
        # get scene
        self.total_step += 1
        if self.total_step % self.checking_interval == 0:

            if IS_IN_ISAAC_SIM:
                purposes = [pxr.UsdGeom.Tokens.default_]
                bboxcache = pxr.UsdGeom.BBoxCache(pxr.Usd.TimeCode.Default(), purposes)
                bboxes = bboxcache.ComputeWorldBound(self.stage.GetPrimAtPath(self.target_prim_path))
                # print("bboxes", bboxes)
                target_bboxes = [bboxes.ComputeAlignedRange().GetMin(), bboxes.ComputeAlignedRange().GetMax()]
                bboxes = bboxcache.ComputeWorldBound(self.stage.GetPrimAtPath(self.container_prim_path))
                # print("bboxes", bboxes)
                container_bboxes = [bboxes.ComputeAlignedRange().GetMin(), bboxes.ComputeAlignedRange().GetMax()]
            else:
                self.context = omni.usd.get_context()
                target_bboxes =  self.context.compute_path_world_bounding_box(self.target_prim_path)
                container_bboxes = self.context.compute_path_world_bounding_box(self.container_prim_path)
                # success condition
            
            target_in_container = self.contains(target_bboxes, container_bboxes)
            
            
            if self.total_step % self.print_every == 0:
                    print("target_in_container", target_in_container)
            if target_in_container == self.in_or_out:
                self.success = True
                self.success_steps += self.checking_interval
            else:
                self._on_not_success()
                # self.timeline.pause()       
            super().start_checking()
        




        

    
