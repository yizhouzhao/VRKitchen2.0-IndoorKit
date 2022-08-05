import os
import json
import math

import pxr 
import omni
import carb

from .base_checker import BaseChecker

from .newContainer_volume_check import liquid_cup_check

class WaterChecker(BaseChecker):
    def __init__(self, task_type, task_id, robot_id, mission_id, annotator, tolerance = 10, has_container = False) -> None:
        super().__init__(task_type, task_id, robot_id, mission_id, annotator)

        # volumn tolerance
        self.tolerance = tolerance  # in percentage
        
        # get target goal condition
        cond = self.current_mission["goal"]["condition"]

        # if has_container, the container would be the target object
        target_obj = cond["target"] if not has_container else  cond["container"]

        self.target_prim_path =  "/World/game/" + target_obj #+ "/Mug"
        self.target_volume = cond["target_value"]
        self.particle_path = "/World/game/Particles"
        
        print("target_prim_path", self.target_prim_path)
        self.record_init_cond()
        print(f"Initialize {self.task_type}, task {self.task_id}, mission {self.mission_id}")
        carb.log_warn("Task desc:" + self.current_mission["goal"]["description"])
        
        # check 1/6 of the steps per second
        self.check_freq = self.steps_per_second / 6

        # volumn shrink if task if "transfer water"
        if self.task_type == "transfer_water":
            self.target_volume *= 0.8
        
    
    def record_init_cond(self):
        from ..param import USE_ISO_SURFACE
        self.liquid_checker = liquid_cup_check(self.target_prim_path, [self.particle_path], USE_ISO_SURFACE)
        self.success_steps = 1 # for printing purpose
    
    def get_percentage(self):
        """
        Get water percentage in original container
        """
        return self.liquid_checker.percentage_inside()
    
    def get_diff(self):
        percentage = self.get_percentage() 
        return percentage / 100 -  self.target_volume
    
    def start_checking(self):
        # success condition
        self.total_step += 1
        if self.total_step % self.checking_interval  == 0:
            percentage = self.get_percentage() 
            print("water percentage", percentage, "delta",  abs(percentage / 100 -  self.target_volume), 
                    "tolerance", (self.tolerance / 100))

            if abs(percentage / 100 -  self.target_volume) < (self.tolerance / 100):
                self.success_steps += self.checking_interval
                self._on_success_hold()
            else:
                self._on_not_success()
                # self.success_steps = 1 # for printing purpose
            super().start_checking()


         
        




        

    
