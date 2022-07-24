from tabnanny import check
from omni.isaac.dynamic_control import _dynamic_control
import omni
import math


class JointCheck():

    def __init__(self, joint_prim, joint_name) -> None:
      
        self.joint_name = joint_name
    
        self.stage = omni.usd.get_context().get_stage()

        self.prim_list = list(self.stage.TraverseAll())
        self.prim_list = [ item for item in self.prim_list if joint_name in  
            item.GetPath().pathString and item.GetPath().pathString.startswith(joint_prim) and item.GetPath().pathString.endswith(joint_name)]

        assert len(self.prim_list) == 1, "len of " + str(len(self.prim_list))
        self.prim = self.prim_list[0]


        self.type = self.prim.GetTypeName()
        self.full_name = self.prim.GetPath().pathString
        self.joint = self.stage.GetPrimAtPath(self.full_name)

        # get joint upper and 
        self.upper = self.joint.GetAttribute("physics:upperLimit").Get()
        self.lower = self.joint.GetAttribute("physics:lowerLimit").Get()

        # need to compute this at the first step
        self.initial_percentage = self.compute_percentage()

    def compute_velocity(self):
        # this function currently is not accurate, do not use it.
        self.dc = _dynamic_control.acquire_dynamic_control_interface()
        self.art = self.dc.get_articulation(self.full_name)

        dof_ptr = self.dc.find_articulation_dof(self.art, self.joint_name)
        dof_vel = self.dc.get_dof_velocity(dof_ptr)
        # dof_vel = self.dc.get_dof_velocity_target(dof_ptr)
        if self.type == 'PhysicsPrismaticJoint':
            from omni.isaac.core.utils.stage import get_stage_units
            v = dof_vel * (get_stage_units() * 100) # in centimeters
            print("units conversion: ", get_stage_units() * 100)
        else:
            v = math.degrees(dof_vel)

        return v
    
    def get_joint_link(self):
        body0 = self.joint.GetRelationship("physics:body0").GetTargets()[0]
        body1 = self.joint.GetRelationship("physics:body1").GetTargets()[0]
        return body1

    def set_velocity(self, velocity):
        self.dc = _dynamic_control.acquire_dynamic_control_interface()
        self.art = self.dc.get_articulation(self.full_name)
        dof_ptr = self.dc.find_articulation_dof(self.art, self.joint_name)

        if self.type == 'PhysicsPrismaticJoint':
            from omni.isaac.core.utils.stage import get_stage_units
            #velocity is in centimeters
            v = velocity / (get_stage_units() * 100) 
        else:
            v = math.radians(velocity)

        self.dc.wake_up_articulation(self.art)
        self.dc.set_dof_velocity(dof_ptr, velocity)
        
    def compute_percentage(self):
        self.dc = _dynamic_control.acquire_dynamic_control_interface()
        self.art = self.dc.get_articulation(self.full_name)

        dof_ptr = self.dc.find_articulation_dof(self.art, self.joint_name)
        dof_pos = self.dc.get_dof_position(dof_ptr)
        
        if self.type == 'PhysicsPrismaticJoint':
            tmp = dof_pos
        else:
            tmp = math.degrees(dof_pos)
        pertentage = (tmp - self.lower)/(self.upper - self.lower) * 100

        # print("upper lower percentage", tmp, self.upper, self.lower, pertentage)

        if pertentage > 100:
            pertentage = 100
        elif pertentage < 0:
            pertentage = 0

        return pertentage 
    
    def compute_distance(self):
        return abs(self.compute_percentage() - self.initial_percentage)

    def set_joint(self, percentage):
        self.dc = _dynamic_control.acquire_dynamic_control_interface()
        self.art = self.dc.get_articulation(self.full_name)
        dof_ptr = self.dc.find_articulation_dof(self.art, self.joint_name)
        upper = self.joint.GetAttribute("physics:upperLimit").Get()
        lower = self.joint.GetAttribute("physics:lowerLimit").Get()
        
        tmp = percentage / 100.0 *(upper-lower) + lower
        if self.type == 'PhysicsPrismaticJoint':
            dof_pos = tmp
        else:
            dof_pos = math.radians(tmp)
      
        self.dc.wake_up_articulation(self.art)
        self.dc.set_dof_position(dof_ptr, dof_pos)
        
#test cases
# check = JointCheck("/World/game/mobility_Door_8897","joint_1")
# check.set_velocity(0.001)
# check.compute_velocity()
# print(check.set_joint(50))

# check = JointCheck("/World/game/mobility_StorageFurniture_40417", "joint_3")
# print(check.compute_velocity())