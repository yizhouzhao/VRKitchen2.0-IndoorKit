# utility functions
import omni
import pxr
from pxr import Gf, Semantics
import carb
import json
import numpy as np

def add_semantics(prim, semantic_label):
    if not prim.HasAPI(Semantics.SemanticsAPI):
        sem = Semantics.SemanticsAPI.Apply(prim, "Semantics")
        sem.CreateSemanticTypeAttr()
        sem.CreateSemanticDataAttr()
        sem.GetSemanticTypeAttr().Set("class")
        sem.GetSemanticDataAttr().Set(semantic_label)

def rename_prim(old_prim_name, new_prim_name):
    # old_prim_name = prim.GetPath().pathString
    # new_prim_name = prim.GetPath().GetParentPath()
    # new_prim_name = new_prim_name.AppendChild("Door1")
    # new_prim_name = omni.usd.get_stage_next_free_path(self.stage, new_prim_name.pathString, False)
    # print("new_prim_name: ", new_prim_name)

    move_dict = {old_prim_name: new_prim_name}
    if pxr.Sdf.Path.IsValidPathString(new_prim_name):
        move_dict = {old_prim_name: new_prim_name}
        omni.kit.commands.execute("MovePrims", paths_to_move=move_dict,  on_move_fn=None)
    else:
        carb.log_error(f"Cannot rename {old_prim_name} to {new_prim_name} as its not a valid USD path")

def freeze_prim(prim, scale = [1, 1, 1]):
    """
    Perform free transform command to current x_form_prim
    """
    stage = omni.usd.get_context().get_stage()
    omni.kit.undo.begin_group()

    prim_name = prim.GetPath().pathString
    temp_name = prim_name + "_temp"
    rename_prim(prim_name, temp_name)
    temp_prim = stage.GetPrimAtPath(temp_name)

    # transform to the correct scale
    prim_xform = Gf.Matrix4d().SetScale(scale)    
    omni.kit.commands.execute(
        "TransformPrimCommand",
        path=temp_name,
        new_transform_matrix=prim_xform,
    )

    # create an unit xform
    omni.kit.commands.execute(
            "CreatePrim",
            prim_path=prim_name,
            prim_type="Xform",
            select_new_prim=False,
        )
    
    

    move_dict = {}
    for prim in temp_prim.GetChildren():
        old_prim_name = prim.GetPath().pathString
        new_prim_name = old_prim_name.replace("_temp", "")
        move_dict[old_prim_name] = new_prim_name
    
    omni.kit.commands.execute("MovePrims", paths_to_move=move_dict,  keep_world_transform = True, on_move_fn=None)

    # print(0/0)

    omni.kit.commands.execute("DeletePrims", paths=[temp_prim.GetPath()])

    # return new root prim
    return stage.GetPrimAtPath(prim_name)



def rotationXYZ_to_quaternion(rotationXYZ):
    translate = Gf.Vec3d(0, 0, 0)
    euler = rotationXYZ
    scale = Gf.Vec3d(1, 1, 1)
    rotation = (
        Gf.Rotation(Gf.Vec3d.ZAxis(), euler[2])
        * Gf.Rotation(Gf.Vec3d.YAxis(), euler[1])
        * Gf.Rotation(Gf.Vec3d.XAxis(), euler[0])
    )
    xform = Gf.Matrix4d().SetScale(scale) * Gf.Matrix4d().SetRotate(rotation) * Gf.Matrix4d().SetTranslate(translate)

    return xform.ExtractRotationQuat()

class NpEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.integer):
            return int(obj)
        if isinstance(obj, np.floating):
            # 👇️ alternatively use str()
            return float(obj)
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)