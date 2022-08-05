# utility function
import re
import omni
import pxr

from ..param import IS_IN_CREAT

def calculate_door_size(prim, scale =  1):
    """
    calculate door size to scale it to the proper size for 3DFront
    """
    target_box_size =  [10, 73.157,  209] # 3D-FRONT door frame size
    if False: #IS_IN_CREAT:
        usd_context = omni.usd.get_context()    
        prim_bboxes = usd_context.compute_path_world_bounding_box(prim.GetPath().pathString)
    # In create
    else:
        purposes = [pxr.UsdGeom.Tokens.default_]
        bboxcache = pxr.UsdGeom.BBoxCache(pxr.Usd.TimeCode.Default(), purposes)
        bboxes = bboxcache.ComputeWorldBound(prim)
        # print("bboxes", bboxes)
        prim_bboxes = [bboxes.ComputeAlignedRange().GetMin(), bboxes.ComputeAlignedRange().GetMax()]
     
    print("prim_bboxes", prim_bboxes)
    s_x = target_box_size[0] / (prim_bboxes[1][0] - prim_bboxes[0][0]) * scale
    s_y = target_box_size[1] / (prim_bboxes[1][1] - prim_bboxes[0][1]) * scale
    s_z = target_box_size[2] / (prim_bboxes[1][2] - prim_bboxes[0][2]) * scale
    
    # if prim_bboxes[1][1] - prim_bboxes[0][1] < prim_bboxes[1][2] - prim_bboxes[0][2]:
    #     s_y, s_z = s_z, s_y
    print("[1, s_y, s_z]", s_x, s_y, s_z)
    return [1, s_y, s_z]
