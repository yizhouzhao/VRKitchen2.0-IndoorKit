# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from click import command
import carb
import omni.ext
import omni.appwindow
import gc
import numpy as np
from omni.isaac.franka import Franka
from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.franka.controllers import RMPFlowController
from omni.isaac.franka.controllers import GripperController
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.utils.stage import add_reference_to_stage, get_stage_units
import copy 
from omni.isaac.core import World
from omni.isaac.core.robots.robot import Robot
from omni.isaac.core.utils.stage import (
    create_new_stage,
    create_new_stage_async,
    get_current_stage,
    set_stage_units,
    set_stage_up_axis,
)
from omni.isaac.core.physics_context import PhysicsContext
# from omni.physx.scripts import physicsUtils
from pxr import UsdGeom, UsdLux, Gf, Vt, UsdPhysics, PhysxSchema, Usd, UsdShade, Sdf
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.core.utils.rotations import euler_angles_to_quat,quat_to_euler_angles, matrix_to_euler_angles
from omni.isaac.synthetic_utils import SyntheticDataHelper
from omni.syntheticdata import sensors
from omni.syntheticdata import visualize
from omni.isaac.core import World
import cv2
import matplotlib.pyplot as plt
import glob
import os
import sys
from omni.kitchen.asset.param import ROBOT_PATH
from omni.kitchen.asset.task_check.graspCheck import GraspCheck
ZERO = [0] * 7


class FrankabotMouse():

    def __init__(self) -> None:
        # self._world = World.instance()
        # if self._world.physics_callback_exists("frankabot_step"):
        #     self._world.remove_physics_callback("frankabot_step")
        self.controling_mode = "start"
        self.pos_gain = 10
        self.rotation_gain = 0.3

    def set_camera_view(eye, target):
        # set_camera_view(eye=np.array([-430, 300, 30]), target=np.array([-420, 70, 130]))
        pass

    def post_load(self, position, rotation, parent_path = None):
        self.app = omni.kit.app.get_app_interface()

        self._controller = None
        self.dof = 3
        self._command = [0] * 7
        self._articulation_controller = None
        self._controller = None
        self.target_positions = None
       
        self.stuck_counter = 0
        self.gripper_open = True

        self.initial_skip_frames = 20
        self.frame_count = 0
    
        self._world = World()
        self.stage = get_current_stage()
        self._timeline = omni.timeline.get_timeline_interface()

        physics_dt = 1.0 / 60.0
        self._world._physics_context = PhysicsContext(physics_dt=physics_dt)
        # self.robot_position = np.array([55, 42, 250])
        
        self._world.clear_all_callbacks()
        self.save_queue = []
        
        robot_path = parent_path + "/franka" if parent_path else "/franka"
        # self._frankabot = Franka(
        #         prim_path = robot_path, name = "my_frankabot",
        #         orientation = rotation,
        #         position = position,
        #     )
        self._frankabot = Franka(
                prim_path = robot_path, name = "my_frankabot",
                usd_path = os.path.join(ROBOT_PATH, "franka/franka.usd"),
                orientation = rotation,
                position = position,
                end_effector_prim_name = 'panda_rightfinger',
                gripper_dof_names = ["panda_finger_joint1", "panda_finger_joint2"],
            )
        self.frame_countdown = 0
        self.target_positions = None
        self.target_rotation = None
        self.skip_frame = 5


        ############## set up images and camera
        # self._set_camera()

        self.record = False
        self.replayBuffer = []
        self.replay_start = False

        ############# set up framerate
        # self._physics_rate = 60
        # carb.settings.get_settings().set_int("/app/runLoops/main/rateLimitFrequency", int(self._physics_rate))
        # carb.settings.get_settings().set_bool("/app/runLoops/main/rateLimitEnabled", True)
        # carb.settings.get_settings().set_int("/persistent/simulation/minFrameRate", int(self._physics_rate))

    def check_grasp(self, object_name = "/World/game/mobility_Bottle_3380"):
        self.grasp_check_handle = GraspCheck(self._controller, self._frankabot.get_articulation_controller(), object_name)
        self.grasp_check_handle.start_check()

    def start_record(self, orig_images_dir, depth_images_dir, traj_dir ):

        self.record = True
        self.orig_images_dir = orig_images_dir
        self.depth_images_dir = depth_images_dir
        self.traj_dir = traj_dir
       
    def stop_record(self ):
        self.record = False

    def replay(self, traj_dir):
       
        import os
        self.traj_dir = traj_dir
        with open(os.path.join(self.traj_dir, 'record.csv'), 'r') as file1:
            Lines = file1.readlines()
        
        self.replayBuffer = [ eval(line) for line in Lines ]

        self.replay_start = True
        




    def _set_camera(self):

        import omni.kit
        from omni.isaac.synthetic_utils import SyntheticDataHelper

        camera_path = ""
        # if self.headless:
        #     viewport_handle = omni.kit.viewport.get_viewport_interface()
        #     viewport_handle.get_viewport_window().set_active_camera(str(camera_path))
        #     viewport_window = viewport_handle.get_viewport_window()
        #     self.viewport_window = viewport_window
        #     viewport_window.set_texture_resolution(128, 128)
        # else:
        viewport_handle = omni.kit.viewport.get_viewport_interface().create_instance()
        new_viewport_name = omni.kit.viewport.get_viewport_interface().get_viewport_window_name(viewport_handle)
        viewport_window = omni.kit.viewport.get_viewport_interface().get_viewport_window(viewport_handle)
        viewport_window.set_active_camera(camera_path)
        viewport_window.set_texture_resolution(300, 300)
        viewport_window.set_window_pos(1000, 400)
        viewport_window.set_window_size(420, 420)
        self.viewport_window = viewport_window

        # self.sd_helper = SyntheticDataHelper()
        # print("try to initialize camera")
        # self.app.update()
        # print("update")
        # self.sd_helper.initialize(sensor_names=["rgb"], viewport=self.viewport_window, timeout = 10)
        
        # sensors.create_or_retrieve_sensor(self.viewport, _syntheticdata.SensorType.Rgb)
        
        # rgb = sensors.get_rgb(self.viewport_window)
        # depth = sensors.get_depth(self.viewport_window)
        # print("finished initializing camera")
        
        # self.sd_helper.get_groundtruth(["rgb"], self.viewport_window)
        return


    def post_setup(self):
        
        self._frankabot.initialize()
        self.gripper_controller = GripperController( name="gripper_controller", gripper_dof_indices = self._frankabot.gripper.dof_indices ) 

        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()
        self._sub_keyboard = self._input.subscribe_to_keyboard_events(self._keyboard, self._sub_keyboard_event)

        self._world.add_physics_callback(callback_name = "frankabot_step", callback_fn=self._on_sim_step)

        if self._controller is None:
            self._controller = RMPFlowController(name="cspace_controller", robot_prim_path = self._frankabot.prim_path )
        self.grip_actions = None

        self._mouse_sub = self._input.subscribe_to_input_events(
                self._on_input_event, order=0
            )
        print("self._mouse_sub", self._mouse_sub)

    def diff(self, arr1, arr2):
        assert len(arr1) == len(arr2)
        diff_value = 0
        for i,j in zip(arr1, arr2):
            diff_value += abs(i - j)

        return diff_value

    def position_control(self):
        
        
        
        if self._controller == None:
            self._controller = RMPFlowController(name="cspace_controller", 
                                   robot_prim_path=self._frankabot.prim_path )

        ee_pos, ee_rot = self._controller._mg.get_end_effector_pose()
        ee_pos *= 100
        ee_rot = matrix_to_euler_angles(ee_rot)
    
        joints_state = self._frankabot.get_joints_state()
        grip_actions = None
        

        if self.frame_countdown == 0:
            if sum(self._command) == 0:
                if self.grip_actions is not None:
                    grip_actions = self.grip_actions
                    self._articulation_controller = self._frankabot.get_articulation_controller()
                    self._articulation_controller.apply_action(grip_actions)
                return 
            if self._command[3] == 1:  
                        
                        grip_actions = self.gripper_controller.forward(
                            action="close", current_joint_positions = joints_state.positions
                        )
                        self.gripper_open = False
                        self.grip_actions = grip_actions
                    
            elif self._command[3] == -1:
                        grip_actions = self.gripper_controller.forward(
                            action="open", current_joint_positions = joints_state.positions
                        )
                        self.gripper_open = True
                        self.grip_actions = None

            self.target_positions = np.array([ee_pos[0] + self._command[0], \
                    ee_pos[1] + self._command[1], ee_pos[2] + self._command[2]])
            
            self.target_rotation = np.array([ee_rot[0] + self._command[4], \
                    ee_rot[1] + self._command[5], ee_rot[2] + self._command[6]])
            

            actions = self._controller.forward(
                        target_end_effector_position = self.target_positions,
                        target_end_effector_orientation = euler_angles_to_quat(self.target_rotation)
                    )
            
            
            if grip_actions is not None:
                for index in self._frankabot.gripper.dof_indices:
                        if grip_actions.joint_positions [index] is None:
                            break
                        actions.joint_positions[index] = grip_actions.joint_positions[index]

            self.frame_countdown = self.skip_frame
        else:
            if self.target_rotation is None or self.target_positions is None:
                return

            
            actions = self._controller.forward(
                        target_end_effector_position = self.target_positions,
                        target_end_effector_orientation = euler_angles_to_quat(self.target_rotation)
                    )
            self.frame_countdown -= 1

        self._articulation_controller = self._frankabot.get_articulation_controller()
        self._articulation_controller.apply_action(actions)
        self._command = [0] * 7

        return actions

    def _on_sim_step(self, step):
        
        

        def get_image_index(save_path):
            return len(glob.glob(save_path + '/*.png'))

        # if sum(self._command) == 0:
        #     return 

        # print("self.replay_start: ", self.replay_start)
        if self.replay_start == False:
            actions = self.position_control()
            # print("actions in control: ", actions)
        else:
            print("len buffer: ", len(self.replayBuffer))
            if len(self.replayBuffer) > 0:

                actions = self.replayBuffer.pop(0)
                if actions is not None:
                    actions = ArticulationAction(joint_positions=actions["joint_positions"])
                    # print("actions: ", actions)
                    self._articulation_controller = self._frankabot.get_articulation_controller()
                    self._articulation_controller.apply_action(actions)
                # print("replay actions: ", actions)

        if self.record == True:
        #     gt = {}
        #     try:
        #         gt["rgb"] = sensors.get_rgb(self.viewport_window)
        #         gt["rgb"] = cv2.cvtColor(gt["rgb"], cv2.COLOR_BGR2RGB)
        #         gt["depth"] = sensors.get_depth(self.viewport_window)
        #         gt["depth"] = visualize.colorize_depth(gt["depth"].squeeze())
        #     except:
        #         return

            # if not os.path.exists(os.path.join(self.traj_dir, 'record.csv')):
            with open(os.path.join(self.traj_dir, 'record.csv'), 'a') as file1:
                    file1.write(str(actions) + '\n')

            # joints_state = self._frankabot.get_joints_state()
            # with open(os.path.join(self.traj_dir, 'joint_state.csv'), 'a') as file1:
            #         file1.write( str(joints_state.positions) + ',' + \
            #             str(joints_state.velocities) + ',' +  str(joints_state.efforts) + '\n')

            # filename = self.orig_images_dir +"/"+ "{0:08d}".format(get_image_index(self.orig_images_dir))  + ".png"
            # cv2.imwrite(filename, gt["rgb"])

            # filename = self.depth_images_dir +"/"+ "{0:08d}".format(get_image_index(self.depth_images_dir)) + ".png"
            # cv2.imwrite(filename, gt["depth"])

        
        
        
     
        return

    def _sub_keyboard_event(self, event, *args, **kwargs):
        """Handle keyboard events
        w,s,a,d as arrow keys for jetbot movement

        Args:
            event (int): keyboard event type
        """
        dof = self.dof
        gain = 10
        rotation_gain = 0.3
        # initial = copy.deepcopy(self._command)
        if self.diff(self._command, [0] * 7 )  > 0:
            return True
        if (
            event.type == carb.input.KeyboardEventType.KEY_PRESS
            or event.type == carb.input.KeyboardEventType.KEY_REPEAT
        ):
        
        # increment
            if event.input == carb.input.KeyboardInput.U:
                self._command[0] += gain

            if event.input == carb.input.KeyboardInput.I:
                self._command[1] += gain


            if event.input == carb.input.KeyboardInput.O:
                self._command[2] += gain
        
        # decrement
            if event.input == carb.input.KeyboardInput.J:
                self._command[0] += -gain

            if event.input == carb.input.KeyboardInput.K:
                self._command[1] += -gain


            if event.input == carb.input.KeyboardInput.L:
                self._command[2] += -gain
            
        #OPEN GRIPPER
            if event.input == carb.input.KeyboardInput.Z:
                self._command[3] = 1

        #CLOSE GRIPPER
            if event.input == carb.input.KeyboardInput.X:
                self._command[3] = -1

        #increment angle
            if event.input == carb.input.KeyboardInput.NUMPAD_7:
                    self._command[4] = rotation_gain

            if event.input == carb.input.KeyboardInput.NUMPAD_8:
                    self._command[5] = rotation_gain

            if event.input == carb.input.KeyboardInput.NUMPAD_9:
                    self._command[6] = rotation_gain

        #decrement angle
            if event.input == carb.input.KeyboardInput.NUMPAD_4:
                    self._command[4] = -rotation_gain

            if event.input == carb.input.KeyboardInput.NUMPAD_5:
                    self._command[5] = -rotation_gain

            if event.input == carb.input.KeyboardInput.NUMPAD_6:
                    self._command[6] = -rotation_gain

        return True

    def unregister_event(self):
        self._input.unsubscribe_to_keyboard_events(self._keyboard, self._sub_keyboard)
        self._input.unsubscribe_to_input_events(self._mouse_sub)


    def _on_input_event(self, event, *_):
        # if not self._viewport.is_focused() or self._viewport.is_manipulating_camera():
        #     return
        if self._timeline.is_playing():
            # if event.deviceType == carb.input.DeviceType.MOUSE:
            #     if event.event.type == carb.input.MouseEventType.MIDDLE_BUTTON_UP:
            #         self.controling_mode = "hand" if self.controling_mode == "robot" else "robot"
            #         print("mouse left button down: now moving ", self.controling_mode)            

                
            if event.deviceType == carb.input.DeviceType.KEYBOARD:
                if event.event.type == carb.input.KeyboardEventType.KEY_RELEASE:
                    if event.event.input == carb.input.KeyboardInput.RIGHT_CONTROL:
                        self.controling_mode = "hand" if self.controling_mode == "robot" else "robot"
                        print("now moving ", self.controling_mode)  
                        
                if event.event.type == carb.input.KeyboardEventType.KEY_PRESS or event.event.type == carb.input.KeyboardEventType.KEY_REPEAT:


                    if event.event.input == carb.input.KeyboardInput.LEFT:
                        if self.controling_mode == "robot":
                            self._command[0] += self.pos_gain
                        else:
                            self._command[4] += self.rotation_gain
                    if event.event.input == carb.input.KeyboardInput.RIGHT:
                        if self.controling_mode == "robot":
                            self._command[0] += -self.pos_gain
                        else:
                            self._command[4] -= self.rotation_gain

                    
                    if event.event.input == carb.input.KeyboardInput.PAGE_UP:
                        if self.controling_mode == "robot":
                            self._command[1] += self.pos_gain
                        else:
                            self._command[5] += self.rotation_gain
                    if event.event.input == carb.input.KeyboardInput.PAGE_DOWN:
                        if self.controling_mode == "robot":
                            self._command[1] += -self.pos_gain
                        else:
                            self._command[5] -= self.rotation_gain

                    if event.event.input == carb.input.KeyboardInput.UP:
                        if self.controling_mode == "robot":
                            self._command[2] += self.pos_gain
                        else:
                            self._command[6] += self.rotation_gain
                    if event.event.input == carb.input.KeyboardInput.DOWN:
                        if self.controling_mode == "robot":
                            self._command[2] += -self.pos_gain
                        else:
                            self._command[6] -= self.rotation_gain

                    #OPEN GRIPPER
                    if event.event.input == carb.input.KeyboardInput.Z:
                        self._command[3] = 1

                    #CLOSE GRIPPER
                    if event.event.input == carb.input.KeyboardInput.X:
                        self._command[3] = -1
            
# FrankabotKeyboard()
