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
import math
from scipy.spatial.transform import Rotation as R

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
from omni.isaac.core.utils.rotations import euler_angles_to_quat, quat_to_euler_angles, matrix_to_euler_angles, quat_to_rot_matrix
from omni.isaac.synthetic_utils import SyntheticDataHelper
from omni.syntheticdata import sensors
from omni.syntheticdata import visualize
from omni.isaac.core.prims import XFormPrim, RigidPrim
import cv2
import matplotlib.pyplot as plt
import glob
import os
import sys
#omni.kit.pipapi.install("transforms3d")
try:
    import transforms3d
except:
    omni.kit.pipapi.install("transforms3d")
    import transforms3d
#omni.kit.pipapi.install("pygame")
try:
    import pygame
except:
    omni.kit.pipapi.install("pygame")
    import pygame

from pygame.locals import *
from omni.kitchen.asset.param import ROBOT_PATH

from omni.isaac.dynamic_control import _dynamic_control

class joystick_handler(object):
    def __init__(self, id):
        self.id = id
        self.joy = pygame.joystick.Joystick(id)
        self.name = self.joy.get_name()
        self.joy.init()
        self.numaxes    = self.joy.get_numaxes()
        self.numballs   = self.joy.get_numballs()
        self.numbuttons = self.joy.get_numbuttons()
        self.numhats    = self.joy.get_numhats()

        self.axis = []
        for i in range(self.numaxes):
            self.axis.append(self.joy.get_axis(i))

        self.ball = []
        for i in range(self.numballs):
            self.ball.append(self.joy.get_ball(i))

        self.button = []
        for i in range(self.numbuttons):
            self.button.append(self.joy.get_button(i))

        self.hat = []
        for i in range(self.numhats):
            self.hat.append(self.joy.get_hat(i))


class FrankabotGamePad():
    ANTI_HUMAN_CONTROL = False
    DISCREET_CONTROL = False
    def __init__(self, target_path, position, rotation, parent_path = None) -> None:
        self.time_step = 0
        self.joy = []
        self.timeline = omni.timeline.get_timeline_interface()
        pygame.init()
        self.joycount = pygame.joystick.get_count()
       
        # assert self.joycount >= 1, "This program only works with at least one joystick plugged in. No joysticks were detected. "
            
        self.joy = []
        for i in range(self.joycount):
            self.joy.append(joystick_handler(i))
        
        self._gamepad_deadzone = 0.15
        # deadzone for pressing right joystick
        self._gamepad_deadzone_large = 0.3
        self.target_path = target_path
        self._controller = None
        
        self.dof = 3
        self._command = [0] * 7
        self._articulation_controller = None
       
        self.stuck_counter = 0
        self.gripper_open = True
        self.can_change_controlling_count_down = True # switch between ee and simple control

        self.initial_skip_frames = 5
        self.frame_count = 0
    
        self.stage = get_current_stage()
        self.save_queue = []
        
        self.robot_path = parent_path + "/franka" if parent_path else "/franka"
       
        self._frankabot = Franka(
                prim_path = self.robot_path, name = "my_frankabot",
                usd_path = os.path.join(ROBOT_PATH, "franka/franka.usd"),
                orientation = rotation,
                position = position,
                end_effector_prim_name = 'panda_rightfinger',
                gripper_dof_names = ["panda_finger_joint1", "panda_finger_joint2"],
            )
        
        self.old_joint_pos = None
        self.joint_thres = 0.025
        self.frame_countdown = 0
        self.target_positions = None
        self.target_rotation = None
        self.skip_frame = 5

        ############## set up images and camera
        self.record = False
        self.replayBuffer = []
        self.replay_start = False

        self.robot_prim = self.stage.GetPrimAtPath(self.robot_path)
        self.game_prim = self.stage.GetPrimAtPath("/World/game")
        self.grasp_state = 0

    def reset_joy(self):
        self.joy = []
        for i in range(self.joycount):
            self.joy.append(joystick_handler(i))

    def _on_timeline_event(self, e):
        """
        set up timeline event
        """
        if e.type == int(omni.timeline.TimelineEventType.STOP):
            self.reset()
    
    def reset(self):
        """
        Reset event
        """
        self._physics_update_subscription = None
        self._timeline_subscription = None


    def start_record(self, orig_images_dir, depth_images_dir, traj_dir ):
        
        self.replay_start = False
        self.record = True
        self.orig_images_dir = orig_images_dir
        self.depth_images_dir = depth_images_dir
        self.traj_dir = traj_dir

        self.create_callback()

        self.time_step = 0
        self.grasp_state = 0

        
       
    def stop_record(self ):
        self.record = False
    
    def create_callback(self):
        stream = self.timeline.get_timeline_event_stream()
        self._timeline_subscription = stream.create_subscription_to_pop(self._on_timeline_event)
        # subscribe to Physics updates:
        self._physics_update_subscription = omni.physx.get_physx_interface().subscribe_physics_step_events(
            self._on_sim_step
        )

    def replay(self, traj_dir):
        
        self.time_step = 0
        
        import os
        self.traj_dir = traj_dir
        with open(os.path.join(self.traj_dir, 'record.csv'), 'r') as file1:
            Lines = file1.readlines()
        
        self.replayBuffer = [ eval(line) for line in Lines ]
        print("replay buffer: ", len(self.replayBuffer))
        self.replay_start = True
        self.create_callback()
        
    def _initialize_control(self):
         # get target object pose and robot pose
        target_obj = XFormPrim(self.target_path)
        obj_trans, obj_rot = target_obj.get_world_pose()
        robot_base = XFormPrim(self.robot_path)
        self.base_trans, base_rot_quat = robot_base.get_world_pose()     
        self.base_rot = R.from_quat(base_rot_quat)  
        vec_base_obj = obj_trans - self.base_trans
    
        # x and z vector in gamepad frame
        z_g, x_g = vec_base_obj[2], vec_base_obj[0]
        ang = np.arctan2(vec_base_obj[0], vec_base_obj[2])

        rot_ang = ang - np.pi/2
        if rot_ang < -np.pi:
            rot_ang += 2*np.pi
        if rot_ang > np.pi:
            rot_ang -= 2*np.pi
        
        # rotation matrix R^{W}_{G}
        self.ang_W_G = rot_ang
        self.R_W_G = np.array([
            [np.cos(rot_ang), -np.sin(rot_ang)], 
            [np.sin(rot_ang), np.cos(rot_ang)]])
        
        ee_pos_init, _ = self._controller._mg.get_end_effector_pose()
        ee_pos_init *= 100
        viewport = omni.kit.viewport.get_viewport_interface()
        viewport_window = viewport.get_viewport_window()
        activeCamera = viewport_window.get_active_camera()
        cameraPos_init = viewport_window.get_camera_position(activeCamera)
        cameraPos_init = list(cameraPos_init)[1:]
        self.camera_ee_diff = np.array(cameraPos_init) - ee_pos_init

    def set_up_controller(self):
        
        self._frankabot.initialize()
        self.gripper_controller = GripperController(name="gripper_controller", gripper_dof_indices = self._frankabot.gripper.dof_indices)
        self._controller = RMPFlowController(name="cspace_controller", robot_prim_path = self._frankabot.prim_path) 
        self._initialize_control()
        self._frankabot.set_joint_positions(self._frankabot._default_joints_state.positions)
        self._frankabot.set_joint_velocities(self._frankabot._default_joints_state.velocities)
        self._frankabot.set_joint_efforts(self._frankabot._default_joints_state.efforts)
        self.joints = ['/World/game/franka/panda_link0/panda_joint1', '/World/game/franka/panda_link2/panda_joint3', 
        '/World/game/franka/panda_link1/panda_joint2', '/World/game/franka/panda_link3/panda_joint4',
         '/World/game/franka/panda_link4/panda_joint5',
        '/World/game/franka/panda_link5/panda_joint6', '/World/game/franka/panda_link6/panda_joint7',  
        '/World/game/franka/panda_link7/panda_joint8',  '/World/game/franka/panda_link8/panda_hand_joint',  
        '/World/game/franka/panda_hand/panda_finger_joint1', '/World/game/franka/panda_hand/panda_finger_joint2']
        for joint in self.joints:
            prim = self.stage.GetPrimAtPath(joint)
            if prim:
                prim.CreateAttribute("physxJoint:maxJointVelocity", Sdf.ValueTypeNames.Float, True).Set(100)

        #if FrankabotGamePad.STUPID_METHOD:
        self.dc = _dynamic_control.acquire_dynamic_control_interface()
        self.art = self.dc.get_articulation("/World/game/franka")
        self.gt = {
            "robot_states": []
        }

        if FrankabotGamePad.DISCREET_CONTROL:
            from ..param import SAVE_ROOT
            save_file_path =os.path.join(SAVE_ROOT, "temp.npy")
            self.robot_target_data = np.load(open(save_file_path, "rb"))
            self.action_count = 0

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

        ee_pos, ee_rot_matrix = self._controller._mg.get_end_effector_pose()
        ee_pos *= 100
        ee_rot = R.from_matrix(ee_rot_matrix)
    
        joints_state = self._frankabot.get_joints_state()
        grip_actions = None

        for event in  pygame.event.get():
                # print("test")
                # QUIT             none
                # ACTIVEEVENT      gain, state
                # KEYDOWN          unicode, key, mod
                # KEYUP            key, mod
                # MOUSEMOTION      pos, rel, buttons
                # MOUSEBUTTONUP    pos, button
                # MOUSEBUTTONDOWN  pos, button
                # JOYAXISMOTION    joy, axis, value
                # JOYBALLMOTION    joy, ball, rel
                # JOYHATMOTION     joy, hat, value
                # JOYBUTTONUP      joy, button
                # JOYBUTTONDOWN    joy, button
                # VIDEORESIZE      size, w, h
                # VIDEOEXPOSE      none
                # USEREVENT        code
                if event.type == QUIT:
                    self.quit()
                elif event.type == KEYDOWN and event.key in [K_ESCAPE, K_q]:
                    self.quit()
                elif event.type == VIDEORESIZE:
                    self.screen = pygame.display.set_mode(event.size, RESIZABLE)
                elif event.type == JOYAXISMOTION:
                    self.joy[event.joy].axis[event.axis] = event.value
                elif event.type == JOYBALLMOTION:
                    self.joy[event.joy].ball[event.ball] = event.rel
                elif event.type == JOYHATMOTION:
                    self.joy[event.joy].hat[event.hat] = event.value
                elif event.type == JOYBUTTONUP:
                    self.joy[event.joy].button[event.button] = 0
                elif event.type == JOYBUTTONDOWN:
                    self.joy[event.joy].button[event.button] = 1

        ########## set end effector position
        def update_ee_position():
            if self.joy[0].button[10] > 0:
                deadzone_to_use = self._gamepad_deadzone_large
            else:
                deadzone_to_use = self._gamepad_deadzone

            robot_transform_matrix = omni.usd.get_local_transform_matrix(self.robot_prim)
            rotation_matrix = robot_transform_matrix.ExtractRotationMatrix()
            
            direction_x = rotation_matrix.GetRow(0)
            direction_y = rotation_matrix.GetRow(1)
            direction_z = rotation_matrix.GetRow(2)

            direction_x = direction_x / np.linalg.norm(direction_x)
            direction_y = direction_y / np.linalg.norm(direction_y)
            direction_z = direction_z / np.linalg.norm(direction_z)

            signal_x = -self.joy[0].axis[4] 
            signal_y = -self.joy[0].axis[3] 
            signal_z = 1/2.0*(self.joy[0].axis[5] - self.joy[0].axis[2])

            self._command[0] = signal_x if abs(signal_x) > deadzone_to_use else 0
            self._command[1] = signal_z if abs(signal_z) > deadzone_to_use else 0
            self._command[2] = signal_y if abs(signal_y) > deadzone_to_use else 0

            self.target_positions = ee_pos + 2 * (self._command[0] * direction_x + self._command[2] * direction_y + self._command[1] * direction_z)

        ########## set camera position using cartesian movement
        def update_camera_1():
            camera_gain = 4
            self._camera_command = [0]*3
            signal_x_camera = -self.joy[0].axis[1]
            signal_z_camera = self.joy[0].axis[0]
            signal_y_camera = self.joy[0].button[5] - self.joy[0].button[4]
            signal_z_tr_cam, signal_x_tr_cam = np.matmul(self.R_W_G, np.array((signal_z_camera, signal_x_camera))).tolist()

            if abs(signal_x_tr_cam) > self._gamepad_deadzone:
                self._camera_command[0] = signal_x_tr_cam * camera_gain 
        
            if abs(signal_z_tr_cam) > self._gamepad_deadzone:
                self._camera_command[2] = signal_z_tr_cam * camera_gain 

            if abs(signal_y_camera) > self._gamepad_deadzone:
                self._camera_command[1] = 1/2.0*signal_y_camera * camera_gain 
            
            disp_new = self.camera_ee_diff + np.array(self._camera_command)
            self.camera_ee_diff = disp_new
            set_camera_view(np.array(ee_pos)+disp_new, ee_pos)

        ########## set camera position using rotation
        def update_camera_2():
            ee_pos_base, _ = self._frankabot.end_effector.get_world_pose()

            ee_pos_updated_camera = [ee_pos_base[i] for i  in range(3) ]

            x, y, z = self.camera_ee_diff
            theta_hor = np.arctan2(x, z)
            theta_vert = np.arctan2(y, np.sqrt(x**2+z**2))
            dist = np.linalg.norm(self.camera_ee_diff)

            camera_gain_angle = 0.1
            camera_gain_dist = 10

            signal_hor = self.joy[0].axis[0]
            signal_vert = -self.joy[0].axis[1]
            signal_dist = self.joy[0].button[4] - self.joy[0].button[5]
            theta_hor_disp = 0
            theta_vert_disp = 0
            dist_disp = 0

            if abs(signal_hor) > self._gamepad_deadzone:
                theta_hor_disp = signal_hor * camera_gain_angle

            if abs(signal_vert) > self._gamepad_deadzone:
                theta_vert_disp = signal_vert * camera_gain_angle

            if abs(signal_dist) > self._gamepad_deadzone:
                dist_disp = 1/2.0*signal_dist * camera_gain_dist

            theta_hor_new = theta_hor + theta_hor_disp
            theta_vert_new = theta_vert + theta_vert_disp
            dist_new = dist + dist_disp

            y_new = np.sin(theta_vert_new)*dist_new
            dist_hor = np.cos(theta_vert_new)*dist_new
            # assert(dist_hor >= 0)
            if dist_hor < 0:
                return

            x_new = np.sin(theta_hor_new)*dist_hor
            z_new = np.cos(theta_hor_new)*dist_hor

            disp_new = np.array((x_new, y_new, z_new))
            self.camera_ee_diff = disp_new
            set_camera_view(np.array(ee_pos_updated_camera)+disp_new, ee_pos_updated_camera)
            

        def update_ee_rot():
            ee_rot_gain = 0.3
       
            signal_x_rot = self.joy[0].button[1] - self.joy[0].button[2]
            if abs(signal_x_rot) > self._gamepad_deadzone:
                self._command[4] = signal_x_rot * ee_rot_gain

            signal_y_rot = self.joy[0].hat[0][0] 
            if abs(signal_y_rot) > self._gamepad_deadzone:
                self._command[5] = signal_y_rot * ee_rot_gain
            
            signal_z_rot = self.joy[0].hat[0][1]
            if abs(signal_z_rot) > self._gamepad_deadzone:
                self._command[6] = signal_z_rot * ee_rot_gain
        
        def update_gripper():
            if self.joy[0].button[3] > 0:
                self.grasp_state = 1

            if self.grasp_state > 0:
                self._command[3] = 1.0
            else:
                self._command[3] = max(self.joy[0].button[10], self.joy[0].button[0])

        update_camera_2()
        update_ee_rot()
        update_gripper()
        update_ee_position()

        # change controlling mode
        if self.joy[0].button[7] > 0 and self.can_change_controlling_count_down == 0:
            FrankabotGamePad.ANTI_HUMAN_CONTROL = True if not FrankabotGamePad.ANTI_HUMAN_CONTROL else False 
            self.can_change_controlling_count_down = 24

        shoulder_gain = -0.4
        if self.frame_countdown == 0:
            # reset controlling option
            if self.can_change_controlling_count_down > 0:
                self.can_change_controlling_count_down -= 1

            if np.sum(np.abs(self._command)) == 0:
                self.target_positions = ee_pos
                self.target_rotation = ee_rot
                return 

            if self._command[3] == 1:
                grip_actions = self.gripper_controller.forward(
                    action="close", current_joint_positions = joints_state.positions
                )
            else:
                grip_actions = self.gripper_controller.forward(
                    action="open", current_joint_positions = joints_state.positions
                )   
            
            rot_disp = R.from_euler('xyz', self._command[4:7])
            base_rot_3d = R.from_euler("y", self.ang_W_G)
            target_rotation_mat = np.matmul(base_rot_3d.as_matrix(), \
                np.matmul(rot_disp.as_matrix(),\
                np.matmul(base_rot_3d.inv().as_matrix(), ee_rot.as_matrix())))
            
            self.target_rotation = R.from_matrix(target_rotation_mat)
            
            if FrankabotGamePad.ANTI_HUMAN_CONTROL:
                actions = self._controller.forward(
                            target_end_effector_position = copy.deepcopy(self.target_positions),
                            target_end_effector_orientation = euler_angles_to_quat(self.target_rotation.as_euler('xyz'))
                        )
            else:
                actions = self._controller.forward(
                        target_end_effector_position = copy.deepcopy(self.target_positions),
                        target_end_effector_orientation = euler_angles_to_quat(ee_rot.as_euler('xyz'))
                    )   

                actions.joint_positions[5] += self._command[6] / 5
                actions.joint_positions[6] += self._command[5]
                actions.joint_positions[0] += shoulder_gain*self._command[4]
                
            if grip_actions is not None:
                for index in self._frankabot.gripper.dof_indices:
                    if grip_actions.joint_positions[index] is None:
                        break
                    actions.joint_positions[index] = grip_actions.joint_positions[index]

            self.frame_countdown = self.skip_frame

            # ignore small joint movements
            if self.old_joint_pos is None:
                self.old_joint_pos = copy.deepcopy(actions.joint_positions)
            if np.linalg.norm(actions.joint_positions - self.old_joint_pos) < self.joint_thres:
                actions.joint_positions = copy.deepcopy(self.old_joint_pos)
            else:
                self.old_joint_pos = copy.deepcopy(actions.joint_positions)
            
        else:
            if self.target_rotation is None or self.target_positions is None:
                self.target_positions = ee_pos
                self.target_rotation = ee_rot
                return
            if FrankabotGamePad.ANTI_HUMAN_CONTROL:
                actions = self._controller.forward(
                            target_end_effector_position = copy.deepcopy(self.target_positions),
                            target_end_effector_orientation = euler_angles_to_quat(self.target_rotation.as_euler('xyz'))
                        )
            else:
                actions = self._controller.forward(
                        target_end_effector_position = copy.deepcopy(self.target_positions),
                        target_end_effector_orientation = euler_angles_to_quat(ee_rot.as_euler('xyz'))
                    )
                actions.joint_positions[5] += self._command[6]
                actions.joint_positions[6] += self._command[5]
                actions.joint_positions[0] += shoulder_gain*self._command[4]
            
            self.frame_countdown -= 1
            # ignore small joint movements
            if self.old_joint_pos is None:
                self.old_joint_pos = copy.deepcopy(actions.joint_positions)
            if np.linalg.norm(actions.joint_positions - self.old_joint_pos) < self.joint_thres:
                actions.joint_positions = copy.deepcopy(self.old_joint_pos)
            else:
                self.old_joint_pos = copy.deepcopy(actions.joint_positions)

        self._articulation_controller = self._frankabot.get_articulation_controller()
        self._articulation_controller.apply_action(actions)
        self._command = [0] * 7
        
        return actions

    def _on_sim_step(self, step):
        if self.time_step == 0:
            self.set_up_controller()
             
        self.time_step += 1

        def get_image_index(save_path):
            return len(glob.glob(save_path + '/*.png'))
       
        if self.replay_start == False:
            actions = self.position_control()
            # print("actions in control: ", actions)
        else:
            dof_states = self.dc.get_articulation_dof_states(self.art, _dynamic_control.STATE_POS)
        
            if len(self.replayBuffer) > 0:
                actions = self.replayBuffer.pop(0)
                if len(self.replayBuffer) % 1000 == 0:
                    print("len buffer: ", len(self.replayBuffer))

                if FrankabotGamePad.DISCREET_CONTROL:
                    self.action_count += 1
                    if self.action_count % 24 == 0:
                        print("self.action_count", self.action_count)
                        joint_actions = self.robot_target_data[self.action_count]
                        t_actions = ArticulationAction(joint_positions=joint_actions)

                        self._articulation_controller = self._frankabot.get_articulation_controller()
                        self._articulation_controller.apply_action(t_actions)

                else:
                    if actions is not None:
                        actions = ArticulationAction(joint_positions=actions["joint_positions"])
                        # print("actions: ", actions)
                        self._articulation_controller = self._frankabot.get_articulation_controller()
                        self._articulation_controller.apply_action(actions)
                    
                if len(self.replayBuffer) == 0:
                    print("Task Done")
                    if self.timeline.is_playing():
                        self.timeline.pause()

                if len(self.replayBuffer) % 120 == 0:
                    print("replay actions: ", actions)
                    print(dof_states["pos"])

                self.gt["robot_states"].append(dof_states["pos"])

        if self.record == True:
            # gt = {}
            # try:
            #     gt["rgb"] = sensors.get_rgb(self.viewport_window)
            #     gt["rgb"] = cv2.cvtColor(gt["rgb"], cv2.COLOR_BGR2RGB)
            #     gt["depth"] = sensors.get_depth(self.viewport_window)
            #     gt["depth"] = visualize.colorize_depth(gt["depth"].squeeze())
            # except:
            #     print("gt error")
            #     # return
            
            # print("recording: ", os.path.join(self.traj_dir, 'record.csv'))
            # if not os.path.exists(os.path.join(self.traj_dir, 'record.csv')):
            with open(os.path.join(self.traj_dir, 'record.csv'), 'a') as file1:
                    file1.write(str(actions) + '\n')

            # joints_state = self._frankabot.get_joints_state()
            # with open(os.path.join(self.traj_dir, 'joint_state.csv'), 'a') as file1:
            #         file1.write( str(joints_state.positions) + ',' + \
            #             str(joints_state.velocities) + ',' +  str(joints_state.efforts) + '\n')

            filename = self.orig_images_dir +"/"+ "{0:08d}".format(get_image_index(self.orig_images_dir))  + ".png"
            # cv2.imwrite(filename, gt["rgb"])

            filename = self.depth_images_dir +"/"+ "{0:08d}".format(get_image_index(self.depth_images_dir)) + ".png"
            # cv2.imwrite(filename, gt["depth"])

        return
