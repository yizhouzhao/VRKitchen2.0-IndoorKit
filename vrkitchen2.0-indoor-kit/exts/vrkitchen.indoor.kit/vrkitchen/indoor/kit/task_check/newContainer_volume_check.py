# from termios import VEOL
import numpy as np

from typing import List, Union
import pxr
from pxr import UsdGeom
import time
import omni
import random
import os
from ..param import IS_IN_ISAAC_SIM

if IS_IN_ISAAC_SIM:
    from omni.isaac.core.prims import XFormPrim

class liquid_cup_check():
    def __init__(self,  cup_path: str, particle_paths: List[str], iso_surface = False) -> None:
        self.cup_path = cup_path
        self.particle_paths = particle_paths
        self.stage = omni.usd.get_context().get_stage()
        print("cup path: ", self.cup_path)
        self.cup_prim = self.stage.GetPrimAtPath(self.cup_path)
        self.cup_xform = UsdGeom.Xformable(self.cup_prim)

        # if iso_surface_paths:
        # print("particle path: ", particle_paths)
        self.iso_surface = iso_surface
        self.iso_surface_path = '/World/game/Fluid/Isosurface'
        # if iso_surface_path:
        #     self.particle_paths = [ str(os.path.join(path,'ellipsoidRaster' )) for path in self.particle_paths]

        # self.inital_particles_count = self.get_particle_positions().shape[0]
        # print(self.percentage_inside())
        # print(self.height_percentage())
        
        # computer cup bounding box
        # self.pre_compute_box_size()

    def pre_compute_box_size(self):    
        purposes = [pxr.UsdGeom.Tokens.default_]
        bboxcache = pxr.UsdGeom.BBoxCache(pxr.Usd.TimeCode.Default(), purposes)

        # if cup_prim getting local transform????????????????????????????????????
        cup_prim = self.stage.GetPrimAtPath(self.cup_path)
        bboxes = bboxcache.ComputeWorldBound(cup_prim)
        prim_bboxes = np.array([bboxes.ComputeAlignedRange().GetMin(), bboxes.ComputeAlignedRange().GetMax()])

        
        self.delta_x = (prim_bboxes[1][0] - prim_bboxes[0][0]) / 2
        self.delta_y = (prim_bboxes[1][1] - prim_bboxes[0][1]) 
        self.delta_z = (prim_bboxes[1][0] - prim_bboxes[0][0]) / 2
        
    
    def get_bbox(self):
        
        bboxes = self.cup_xform.ComputeWorldBound(0, UsdGeom.Tokens.default_ )
        prim_bboxes = np.array([bboxes.ComputeAlignedRange().GetMin(), bboxes.ComputeAlignedRange().GetMax()])

        # mat = omni.usd.utils.get_world_transform_matrix(cup_prim) 
        # translation = mat.ExtractTranslation()
        
        # prim_bboxes = prim_bboxes + translation
        # print("prim_bboxes2", self.cup_path, prim_bboxes)
        return prim_bboxes

    def height_percentage(self):
        #from omni.isaac.core.utils.stage import get_stage_up_axis
        particle_positions = self.get_particle_positions()
        up_axis = pxr.UsdGeom.Tokens.y # get_stage_up_axis()
        axises = [pxr.UsdGeom.Tokens.x, pxr.UsdGeom.Tokens.y, pxr.UsdGeom.Tokens.z]
        box = self.get_bbox()
   
        def inside(particle):
            inA = particle[0] > box[0][0] and particle[0] < box[1][0]
            inB = particle[1] > box[0][1] and particle[1] < box[1][1]
            inC = particle[2] > box[0][2] and particle[2] < box[1][2]
            if inA and inB and inC:
                return True
            return False
        
        res = list(map(inside, particle_positions))
        if not np.any(res):
            return 0

        inside_particles_max = np.max([particle_position[axises.index(up_axis)] for particle_position in particle_positions[res]])
        
        height_percentage = (inside_particles_max-box[0][ axises.index(up_axis) ])/( box[1][ axises.index(up_axis)]- box[0][ axises.index(up_axis)])
        return height_percentage * 100.0
    
    def percentage_inside(self):
        
        particle_positions = self.get_particle_positions().tolist()
        box = self.get_bbox()

        def inside(particle):
            inA = particle[0] > box[0][0] and particle[0] < box[1][0]
            inB = particle[1] > box[0][1] and particle[1] < box[1][1]
            inC = particle[2] > box[0][2] and particle[2] < box[1][2]
            if inA and inB and inC:
                return True
            return False

        res = list(map(inside, particle_positions))    
        return sum(res)/len(res) * 100.0

    def get_particle_positions(self, paths :Union[None, List[str] ] = None ):
        positions = []
        if paths is not None:
            path_in_use = paths
        else:
            path_in_use = self.particle_paths

        for particle_path in path_in_use:
            tmp = self.get_particle_position_list(particle_path=particle_path)
            positions.append(tmp)
        
        particle_positions = np.vstack(positions)
        return particle_positions
    
    def get_all_particles(self, paths :Union[None, List[str] ] = None ):
        ptcl_dict = {}
        if paths is not None:
            path_in_use = paths
        else:
            path_in_use = self.particle_paths

        for particle_path in path_in_use:
            particle_prim = self.stage.GetPrimAtPath(particle_path)
            particles = pxr.UsdGeom.PointInstancer(particle_prim)
            pos = np.around(np.array(particles.GetPositionsAttr().Get()).astype(np.longdouble), 3).tolist()
            vel = np.around(np.array(particles.GetVelocitiesAttr().Get()).astype(np.longdouble), 3).tolist()
            ptcl_dict[particle_path] = [pos, vel]
        
        return ptcl_dict

    def get_particle_position_list(self, particle_path):
        if self.iso_surface:
            from omni.isaac.core.utils.prims import get_prim_at_path
            # self.particle_prim = self.stage.GetPrimAtPath(self.iso_surface_path)
            # mat = omni.usd.utils.get_world_transform_matrix(self.particle_prim) 
            # translation = mat.ExtractTranslation()
            # rotation_matrix = mat.ExtractRotationMatrix()

            iso_path = str(os.path.join( self.iso_surface_path ))
            # print("iso_path: ", iso_path)
            positions = get_prim_at_path(iso_path).GetAttribute("points").Get()
            positions = np.array(positions)
            # positions = positions[:,:3]
            # print("positions: ", positions)
            # print("translation", translation)
        else:
            self.particle_prim = self.stage.GetPrimAtPath(particle_path)
            mat = omni.usd.utils.get_world_transform_matrix(self.particle_prim) 
            translation = mat.ExtractTranslation()
            rotation_matrix = mat.ExtractRotationMatrix()
            particles = pxr.UsdGeom.PointInstancer(self.particle_prim)
            positions = np.array(particles.GetPositionsAttr().Get())
            positions = positions @  rotation_matrix + translation

        return positions #+ translation
    
    def set_all_particles(self, ptcl_dict):
        path_in_use = list(ptcl_dict.keys())
        for particle_path in path_in_use:
            if particle_path in self.particle_paths:
                particle_prim = self.stage.GetPrimAtPath(particle_path)
                particles = pxr.UsdGeom.PointInstancer(particle_prim)
                pos = ptcl_dict[particle_path][0]
                particles.CreatePositionsAttr().Set([pxr.Gf.Vec3f(*p) for p in pos])
                vel = ptcl_dict[particle_path][1]
                particles.CreateVelocitiesAttr().Set([pxr.Gf.Vec3f(*v) for v in vel])

# #test
# a = time.time()
# liquid_cup_check("/World/Mug", ["/World/Particles"] )
# b = time.time()
# print(b-a)