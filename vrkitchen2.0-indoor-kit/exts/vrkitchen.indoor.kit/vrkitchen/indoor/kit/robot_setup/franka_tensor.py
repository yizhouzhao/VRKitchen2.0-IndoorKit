import omni
import carb
import types
import numpy as np
import importlib
import os

from ..param import IS_IN_CREAT, APP_VERION
from .controller import Controller
from .numpy_utils import orientation_error
    
from pxr import Usd, UsdGeom, Gf

if APP_VERION >= "2022.1.0":
    class FrankaTensor():
        def __init__(self, enable_tensor_api = True) -> None:
            carb.log_info("Franks Tensor started (only in Create >= 2022.1.1)")
            self._is_stopped = True
            self._tensor_started = False
            self._tensor_api = None
            self._flatcache_was_enabled = True
            self._tensorapi_was_enabled = True

            # stage
            self.stage = omni.usd.get_context().get_stage()
            self.franka_prim = self.stage.GetPrimAtPath("/World/game/franka")

            # property
            self.is_replay = False

            # counting and index 
            self.count_down = 120
            self.button_status = 0 
            self.npz_index = 0
            self.is_start = True

            # setup subscriptions:
            self._setup_callbacks()
            if enable_tensor_api:
                self._enable_tensor_api()

            # controller
            self.controller = Controller()

        def _enable_tensor_api(self):
                manager = omni.kit.app.get_app().get_extension_manager()
                self._tensorapi_was_enabled = manager.is_extension_enabled("omni.physx.tensors")
                if not self._tensorapi_was_enabled:
                    manager.set_extension_enabled_immediate("omni.physx.tensors", True)
                self._tensor_api = importlib.import_module("omni.physics.tensors")

        # "PRIVATE" METHODS #
        def _can_callback_physics_step(self) -> bool:
            if self._is_stopped:
                return False

            if self._tensor_started or self._tensor_api is None:
                return True

            self._tensor_started = True
            self.on_tensor_start(self._tensor_api)
            return True
            
        def on_tensor_start(self, tensorApi: types.ModuleType):
            """
            This method is called when
                1. the tensor API is enabled, and
                2. when the simulation data is ready for the user to setup views using the tensor API.
            """
            # if IS_IN_CREAT and APP_VERION >= "2022.1.1":
            sim = tensorApi.create_simulation_view("numpy")
            sim.set_subspace_roots("/World/game/*")
            # franka view
            self.frankas = sim.create_articulation_view("/World/game/franka")
            self.franka_indices = np.arange(self.frankas.count, dtype=np.int32)

            # !!!
            # self.default_dof_pos = np.array([0.0, 0.0, 0.0, -0.95, 0.0, 1.12, 0.0, 0.02, 0.02])
            self.default_dof_pos = np.array([1.2024134e-02, -5.6960440e-01, 7.3155526e-05, -2.8114836e+00,
                -4.8544933e-03,  3.0270250e+00,  7.2893953e-01,  3.9919264e+00, 4.0000000e+00])

            # set default dof pos:
            init_dof_pos = np.stack(1 * [np.array(self.default_dof_pos, dtype=np.float32)])
            self.frankas.set_dof_position_targets(init_dof_pos, self.franka_indices)
            self.last_gripper_action = 1 # open as default

            # end effector view
            self.hands = sim.create_rigid_body_view("/World/game/franka/panda_hand")

            # get initial hand transforms
            # init_hand_transforms = self.hands.get_transforms().copy()
            # self.hand_pos = init_hand_transforms[:, :3]
            # self.hand_rot = init_hand_transforms[:, 3:]

            
            # target 
            # self.target_pos = self.default_dof_pos[None, :]
            # self.target_hand_transform = init_hand_transforms

    
        def _setup_callbacks(self):
            stream = omni.timeline.get_timeline_interface().get_timeline_event_stream()
            self._timeline_sub = stream.create_subscription_to_pop(self._on_timeline_event)
            
            # subscribe to Physics updates:
            self._physics_update_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(self._on_physics_step)
            events = omni.physx.get_physx_interface().get_simulation_event_stream_v2()
            self._simulation_event_subscription = events.create_subscription_to_pop(self.on_simulation_event)

            # subscribute to keyboard 
            self._appwindow = omni.appwindow.get_default_app_window()
            self._input = carb.input.acquire_input_interface()
            self._keyboard = self._appwindow.get_keyboard()
            self._sub_keyboard = self._input.subscribe_to_keyboard_events(self._keyboard, self._sub_keyboard_event)

        def _sub_keyboard_event(self, event, *args, **kwargs):
            self.controller.handle_keyboard_event(event)
    
        def _on_timeline_event(self, e):
            if e.type == int(omni.timeline.TimelineEventType.STOP):
                self._is_stopped = True
                self._tensor_started = False

                # !!!
                self._timeline_sub = None
                self._simulation_event_subscription = None
                self._physics_update_sub = None

                self._input.unsubscribe_to_keyboard_events(self._keyboard, self._sub_keyboard)


            if e.type == int(omni.timeline.TimelineEventType.PLAY):
                self._is_stopped = False

            # call user implementation
            # self.on_timeline_event(e)
        
        def _on_physics_step(self, dt):
            if not self._can_callback_physics_step():
                return

            # call user implementation
            self.on_physics_step(dt)

        def on_simulation_event(self, e):
            """
            This method is called on simulation events. See omni.physx.bindings._physx.SimulationEvent.
            """
            pass

        
        def on_physics_step(self, dt):
            """
            This method is called on each physics step callback, and the first callback is issued
            after the on_tensor_start method is called if the tensor API is enabled.
            """
            self.count_down -= 1
            # self.dof_pos = self.frankas.get_dof_positions()
            # print("dof_pos", self.dof_pos)
        
            # playing 
            if not self.is_replay:
                if self.count_down == 0:

                    self.count_down = 6 # TODO: unify count_down is play and replay
                    
                    # get movement from keyboard 
                    move_vec = self.controller.QueryMove()
                    query_move =  move_vec != [0, 0, 0]
                    
                    # get rotation from keyboard 
                    rotation_vec = self.controller.QueryRotation()
                    query_rotation = rotation_vec != [0, 0]

                    # get gripper
                    gripper_val = self.controller.QueryGripper()
                    query_gripper =  self.last_gripper_action != gripper_val

                    # get end effector transforms
                    hand_transforms = self.hands.get_transforms().copy()
                    current_hand_pos, current_hand_rot = hand_transforms[:, :3], hand_transforms[:, 3:]

                    # update record
                    if query_move or query_rotation or query_gripper or self.is_start:
                        self.hand_pos = current_hand_pos
                        self.hand_rot = current_hand_rot
                        self.last_gripper_action = gripper_val

                        self.is_start = False

                    # print("current_dof_pos", self.frankas.get_dof_positions())

                    # # if no input 
                    # if not query_move and not query_rotation and not query_gripper:
                    #     return 

                    # get franka xform mat # FIXME: time code?
                    mat = UsdGeom.Xformable(self.franka_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                    
                    move_vec_4d = Gf.Vec4d(move_vec[0], move_vec[1], move_vec[2], 0)
                    hand_move = move_vec_4d * mat
                    hand_move_np = np.array([[hand_move[0], hand_move[1], hand_move[2]]])

                    target_pos = self.hand_pos + hand_move_np
                    target_rot = self.hand_rot

                    dof_target = self.move_to_target(target_pos, target_rot)

                    if query_rotation:
                        dof_target[...,5] += rotation_vec[0] * 0.1 # slowly but surely
                        dof_target[...,6] += rotation_vec[1] * 0.2

                    # print("last_gripper_action", self.last_gripper_action)
                    dof_target[...,[-2, -1]] = 5 if self.last_gripper_action > 0 else -1

                    self.frankas.set_dof_position_targets(dof_target, np.arange(1))

            # replaying
            else: # self.is_replay:
                np_root = "E:/researches/VRKitchen2.0/learning/vrkit_data/AWS-pickup_object-0-0-0-8-0-0/trajectory/"

                if self.count_down == 0:
                    npz_path = np_root + f"{(self.npz_index * 24):08d}.npz"

                    # pause when npz not exist
                    if not os.path.exists(npz_path):
                        omni.timeline.get_timeline_interface().pause()
                        return

                    robot_pos = np.load(npz_path)['robot_state']['pos']
                    print("robot_pos", robot_pos)
                    self.count_down = 24
                    self.npz_index += 1

                    self.target_pos = robot_pos[None, :]
                    self.frankas.set_dof_position_targets(self.target_pos, self.franka_indices)

        ######################################### robot control #########################################


        def move_to_target(self, goal_pos, goal_rot):
            """
            Move hand to target points
            """
            # get end effector transforms
            hand_transforms = self.hands.get_transforms().copy()
            hand_pos, hand_rot = hand_transforms[:, :3], hand_transforms[:, 3:]
            #hand_rot = hand_rot[:,[1,2,3,0]] # WXYZ

            # get franka DOF states
            dof_pos = self.frankas.get_dof_positions()

            # compute position and orientation error
            pos_err = goal_pos - hand_pos
            orn_err = orientation_error(goal_rot, hand_rot)
            dpose = np.concatenate([pos_err, orn_err], -1)[:, None].transpose(0, 2, 1)

            jacobians = self.frankas.get_jacobians()

            # jacobian entries corresponding to franka hand
            franka_hand_index = 8  # !!!
            j_eef = jacobians[:, franka_hand_index - 1, :]

            # solve damped least squares
            j_eef_T = np.transpose(j_eef, (0, 2, 1))
            d = 0.05  # damping term
            lmbda = np.eye(6) * (d ** 2)
            u = (j_eef_T @ np.linalg.inv(j_eef @ j_eef_T + lmbda) @ dpose).reshape(1, 9)

            # update position targets
            pos_targets = dof_pos + u  # * 0.3

            return pos_targets