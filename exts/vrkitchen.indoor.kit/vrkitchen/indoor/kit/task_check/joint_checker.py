import os
import json
import pxr 
import omni
import carb
from .base_checker import BaseChecker

from ..param import IS_IN_ISAAC_SIM

if IS_IN_ISAAC_SIM:
    from .newJointCheck import JointCheck

class JointChecker(BaseChecker):
    def __init__(self, task_type, task_id, robot_id, mission_id, annotator="Steven", tolerance = 5) -> None:
        super().__init__(task_type, task_id, robot_id, mission_id, annotator)

        self.tolerance = tolerance
        
        assert len(self.current_mission) > 0, "mission needs to exist"
        # if len(self.current_mission) == 0:
        #     carb.log_warn(f"current mission must be specified at path {self.mission_file_path} and mission id {self.mission_id}")

        # get target goal condition
        self.cond = self.current_mission["goal"]["condition"]
        self.target_obj = self.cond["target"]
        self.target_joint = self.cond["joint"]

        self.init_value = self.cond["init_value"] if "init_value" in self.cond else -1
        self.target_value = self.cond["target_value"]
        self.target_prim_path = "/World/game/" + self.target_obj
        if IS_IN_ISAAC_SIM:
            self.joint_checker = JointCheck(self.target_prim_path, self.target_joint)

            if self.task_type in ["open_cabinet", "close_cabinet"]:
                self.check_joint_direction()
        
        carb.log_warn("Task desc:" + self.current_mission["goal"]["description"])
        
        # set joint at start
        self.set_joint_at_start = True if self.init_value != -1 else False

        self.previous_percentage = None
        self.vel = None
        self.checking_interval = 1
    
    def check_joint_direction(self):
        """
        Check joint positive rotation to upper or negative rotation to lower
        """
        is_upper = abs(self.joint_checker.upper) > abs(self.joint_checker.lower)
        if not is_upper:
            # if is lower, reverse init_value and target value
            self.init_value = 1 - self.init_value if self.init_value != -1 else -1
            self.target_value = 1 - self.target_value

    def get_diff(self):
        percentage =  self.joint_checker.compute_percentage()
        return percentage / 100.0 - self.target_value
        
    def start_checking(self):
        
        self.total_step += 1
        if self.total_step % self.checking_interval == 0:
            if IS_IN_ISAAC_SIM:
                if self.set_joint_at_start:
                    self.joint_checker.set_joint(self.init_value * 100)
                    self.set_joint_at_start = False

                percentage =  self.joint_checker.compute_percentage()
            
                if self.previous_percentage is not None:
                    self.vel  = abs(percentage - self.previous_percentage)
                    # print("self.vel: ", self.vel)

                # log
                if self.total_step % self.print_every == 0:
                    print("current: {:.1f}; target: {:.1f}; delta percentage: {:.1f}:".format(percentage, self.target_value * 100, self.target_value * 100 - percentage) )
                    
                
                if abs(percentage / 100.0 - self.target_value) < self.tolerance/100.0 :
                    self.success_steps += self.checking_interval
                    self._on_success_hold()
                    # print("success pending")
                else:
                    self._on_not_success()

                self.previous_percentage = percentage
                
                super().start_checking()