import math
import numpy as np

import pxr 
import omni
import carb

from .base_checker import BaseChecker

from ..param import IS_IN_ISAAC_SIM

if IS_IN_ISAAC_SIM:
    from .newContainer_volume_check import liquid_cup_check



class TapWaterChecker(BaseChecker):
    def __init__(self, task_type, task_id, robot_id, mission_id, annotator, tolerance = 10, faucet=None) -> None:
        super().__init__(task_type, task_id, robot_id, mission_id, annotator)

        # volumn tolerance
        self.tolerance = tolerance / 100 # in percentage
        
        # get target goal condition
        print(task_type, task_id, robot_id, mission_id, annotator)
        print("self.current mission: ", self.current_mission)
        cond = self.current_mission["goal"]["condition"]
        container_obj = cond["container"]

        self.volume_prim_path =  "/World/game/" + container_obj #+ "/cup_volume"
        self.target_volume = cond["target_value"]
        # self.particle_path = "/World/game/" + target_obj+ "/Particles"
        
        print("volumn_prim_path", self.volume_prim_path)
        self.record_init_cond()
        print(f"Initialize {self.task_type}, task {self.task_id}, mission {self.mission_id}")
        carb.log_warn("Task desc:" + self.current_mission["goal"]["description"])

        self.faucet = faucet
    
    def set_faucet(self, faucet):
        self.faucet = faucet

    def modify_mission(self):
        """
        modify mission acorrding to task type and scene
        """
        pass
    
    def record_init_cond(self):
        self.stage = omni.usd.get_context().get_stage()
        if IS_IN_ISAAC_SIM:
            self.liquid_checker = liquid_cup_check(self.volume_prim_path, [])
   
    def update_particle_paths(self, horizon=0):
        # if IS_IN_ISAAC_SIM:
        particlesScope = self.stage.GetPrimAtPath("/World/game/inflow")
        if particlesScope:
            particlesPathList = [prim.GetPath().pathString for prim in particlesScope.GetChildren()]
            if horizon == 0:
                # calculate all particles
                self.liquid_checker.particle_paths = particlesPathList
            else:
                # only calculate most recent ones
                self.liquid_checker.particle_paths = particlesPathList[:horizon]
            
    
    def get_percentage(self):
        """
        Get water percentage in original container
        """
        self.update_particle_paths()
        if len(self.liquid_checker.particle_paths) > 0:
            return self.liquid_checker.height_percentage()
        else:
            return 0.0
    
    def get_diff(self):
        percentage = self.get_percentage() 
        return percentage / 100 -  self.target_volume
    
    def start_checking(self):
        # success condition
        self.total_step += 1
        if self.total_step % self.checking_interval == 0:
            percentage = self.get_percentage() 
            if self.total_step % self.print_every == 0:
                print("water percentage", percentage)

            if abs(percentage / 100 -  self.target_volume) < self.tolerance and self.faucet.is_off():
                self.success_steps += self.checking_interval
                self._on_success_hold()
            else:
                self._on_not_success()
            super().start_checking()
         
        




        

    
