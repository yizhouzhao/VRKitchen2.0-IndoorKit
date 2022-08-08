import omni
import pxr
from pxr import Gf
import carb

import os
import random
import json

from omni.kit.material.library import get_material_prim_path, create_mdl_material

from ..param import IS_IN_ISAAC_SIM, SAPIEN_ASSET_PATH, HOUSE_INFO_PATH, DATA_PATH

class Randomizer():
    def __init__(self, task_json_path=None, random_seed = 1) -> None:
        # self.house = house
        # self.layout = self.house.layout if house is not None else {}
        self.task_json_path = task_json_path
        self.random_seed = random_seed

        # randomize index
        self.light_rnd = -1 # light randomized index
        self.location_rnd = -1 # game loc randomized index
        self.material_rnd = -1 # material randomized index
        if task_json_path:
            if not os.path.exists(self.task_json_path):
                raise Exception( "The json file at path {} provided wasn't found".format(self.task_json_path))

            self.task_json = json.load(open(self.task_json_path))
        else:
            self.task_json = {}
        # init randomization
        if "random" not in self.task_json:
            self.random_info = {
                "lights":[],
                "materials":{},
                "locations":[{
                    "translate":[0,0,0],
                    "orient":[1,0,0,0],
                    "scale":[1.0,1.0,1.0]
                }],
            }

            self.task_json["random"] = self.random_info
        else:
            self.random_info = self.task_json["random"]

        # material
        self.material_dict = {}
        

    # @staticmethod
    def get_water_material(self):
        from pxr import Tf, Sdf, Usd, UsdShade
        # self.setup_material_helper()
        # print()
        water_url = 'http://localhost:8080/omniverse://127.0.0.1/NVIDIA/Materials/Base/Natural/Water.mdl'
        water_mtl_name = water_url.split("/")[-1][:-4]
        # print("material dict: ", self.material_dict)
        water_material_prim_path = get_material_prim_path(water_mtl_name)

        # omni.kit.commands.execute(
        #         "CreatePrim", prim_path=water_material_prim_path, prim_type="Scope", select_new_prim=False
        #     )

        def on_create(path):
            pass
        return create_mdl_material(omni.usd.get_context().get_stage(), water_url, water_mtl_name, on_create)
        # stage = omni.usd.get_context().get_stage()

        # if stage.HasDefaultPrim():
        #     mtl_path = omni.usd.get_stage_next_free_path(
        #         stage, "{}/Looks/{}".format(stage.GetDefaultPrim().GetPath(), Tf.MakeValidIdentifier(water_mtl_name)), False
        #     )
        # else:
        #     mtl_path = omni.usd.get_stage_next_free_path(
        #         stage, "/Looks/{}".format(Tf.MakeValidIdentifier(water_mtl_name)), False
        #     )

        # omni.kit.commands.execute("CreateMdlMaterialPrim", mtl_url=water_url, mtl_name=water_mtl_name, 
        #     mtl_path=water_material_prim_path, select_new_prim=False)

        # return water_material_prim_path
        # omni.kit.commands.execute(
        #     "CreateMdlMaterialPrim",
        #     mtl_url=water_url,
        #     mtl_name=water_mtl_name,
        #     mtl_path=water_material_prim_path,
        #     select_new_prim=False,
        # )

        # omni.kit.commands.execute(
        #     'BindMaterial',
        #     prim_path=prim.GetPath(),
        #     material_path = water_material_prim_path,
        #     strength=pxr.UsdShade.Tokens.strongerThanDescendants
        # )
        return water_material_prim_path



        

    def set_seed(self, seed):
        self.random_seed = seed

    def randomize_light(self):
        """
        Randomize light intensity
        """
        self.random_info["lights"] = [0, 200, 400, 600, 800, 1000] # light intensity indexes
        self.light_rnd = random.choice([_ for _ in range(len(self.random_info["lights"]))])
        
        self.stage = omni.usd.get_context().get_stage()
        self.default_prim = self.stage.GetDefaultPrim()
        # print("?", self.default_prim.GetPath().pathString + "/defaultLight")
        light_prim = self.stage.GetPrimAtPath(self.default_prim.GetPath().pathString + "/defaultLight")
        assert light_prim.GetTypeName() == "DistantLight"
        light_prim.GetAttribute("intensity").Set(self.random_info["lights"][self.light_rnd])


    def randomize_game_location(self):
        """
        Randomize light intensity
        """
        assert len(self.random_info["locations"]) > 0
        
        self.location_rnd = (self.location_rnd + 1) % len(self.random_info["locations"])

        self.stage = omni.usd.get_context().get_stage()
        self.default_prim = self.stage.GetDefaultPrim()
        game_prim = self.stage.GetPrimAtPath(self.default_prim.GetPath().pathString + "/game")

        game_layout = self.random_info["locations"][self.location_rnd]
        assert "translate" in game_layout and "orient" in game_layout
        
        translate = game_layout["translate"]
        orient =  game_layout["orient"]
        rotation = Gf.Quatd(orient[0], orient[1], orient[2], orient[3])

        # TODO: check whether scale can be randomized
        scale = (1.0, 1.0, 1.0) 

        print("location")
        xform = Gf.Matrix4d().SetScale(scale) * Gf.Matrix4d().SetRotate(rotation) * Gf.Matrix4d().SetTranslate(translate)
        omni.kit.commands.execute(
            "TransformPrimCommand",
            path=game_prim.GetPath(),
            new_transform_matrix=xform,
        )

    def setup_material_helper(self):
        """
        set up material randomizer
        """
        self.stage = omni.usd.get_context().get_stage()
        # check if has material
        if len(self.material_dict) > 0:
            return 
        
        carb.log_info("loading necleu materials")

        # load from saved params
        try:
            # load the materials from nucleus url link
            mat_root_path = "http://localhost:8080/omniverse://127.0.0.1/NVIDIA/Materials/"
            carb.log_info(f"Collecting files for {mat_root_path}")
            result1, entries = omni.client.list(mat_root_path)

            from .material.param import NECLEUS_MATERIALS
            self.material_dict = NECLEUS_MATERIALS
        except:
            
            # load the materials from nucleus url link
            mat_root_path = "http://localhost:8080/omniverse://127.0.0.1/NVIDIA/Materials/"
            carb.log_info(f"Collecting files for {mat_root_path}")
            result1, entries = omni.client.list(mat_root_path)
        
            if result1 != omni.client.Result.OK:
                raise Exception(f"nucleus connect error at path: {mat_root_path}")
            for e in entries:
                print("result: ", e.relative_path)
                material_type_folder = mat_root_path + e.relative_path + "/"
                result2, mat_type_entries = omni.client.list(material_type_folder)
                for mat_type_e in mat_type_entries:
                    if mat_type_e.relative_path not in self.material_dict:
                        self.material_dict[mat_type_e.relative_path] = []
                    material_folder = material_type_folder + mat_type_e.relative_path + "/"
                    result3, mat_entries = omni.client.list(material_folder)
                    for mat_e in mat_entries:
                        if mat_e.relative_path.endswith(".mdl"):
                            mat_path = material_folder + mat_e.relative_path
                            self.material_dict[mat_type_e.relative_path].append(mat_path)

            # filter_out_empty
            temp_dict = {}
            for key in self.material_dict:
                if len(self.material_dict[key]) > 0:
                    temp_dict[key] = self.material_dict[key]
                    
            self.material_dict = temp_dict
        # mtl_created_list = []
            # omni.kit.commands.execute(
            #     "CreateAndBindMdlMaterialFromLibrary",
            #     mdl_name='http://localhost:8080/omniverse://127.0.0.1/NVIDIA/Materials/Base/Architecture/Ceiling_Tiles.mdl',
            #     mtl_name='Ceiling_Tiles',
            #     mtl_created_list=mtl_created_list,
            #     bind_selected_prims=True,
            #     select_new_prim=False,
            # )
    
    def randomize_house(self, rand = True, randomize_floor =True, randomize_wall = True):
        """
        randomize house's floor and wall
        
        by default, we only randomize floor
        """

        self.setup_material_helper()
        floor_parent = self.stage.GetPrimAtPath("/World/layout/floors")
        wall_parent = self.stage.GetPrimAtPath("/World/layout/roomStruct")
        self.random_info["floor_materials"] = [x for k in ["Wood"] for x in self.material_dict[k]] # Carpet
        self.random_info["wall_materials"] = [x for k in ["Wall_Board"] for x in self.material_dict[k]] # "Masonry", "Architecture"
        # print(self.random_info["floor_materials"])
        # len_floor = len(self.random_info["floor_materials"])
        # len_wall = len(self.random_info["wall_materials"])
        wall_mtl_url = random.choice(self.random_info["wall_materials"]) if rand else self.random_info["wall_materials"][0]
        floor_mtl_url = random.choice(self.random_info["floor_materials"]) if rand else self.random_info["floor_materials"][0]
        wall_mtl_name = wall_mtl_url.split("/")[-1][:-4]
        floor_mtl_name = floor_mtl_url.split("/")[-1][:-4]

        # change mtl
        new_looks_path1, wall_material_prim_path = get_material_prim_path(wall_mtl_name)
        if new_looks_path1 and randomize_wall:
            omni.kit.commands.execute(
                "CreatePrim", prim_path=new_looks_path1, prim_type="Scope", select_new_prim=False
            )
        new_looks_path2, floor_material_prim_path = get_material_prim_path(floor_mtl_name)
        if new_looks_path2 and randomize_floor:
            omni.kit.commands.execute(
                "CreatePrim", prim_path=new_looks_path2, prim_type="Scope", select_new_prim=False
            )
        
        for prim in floor_parent.GetChildren():
            if prim is None:
                raise Exception("no house in scene!")

            carb.log_info("changing material at path: " + prim.GetPath().pathString)

            if floor_material_prim_path:
                omni.kit.commands.execute(
                    "CreateMdlMaterialPrim",
                    mtl_url=floor_mtl_url,
                    mtl_name=floor_mtl_name,
                    mtl_path=floor_material_prim_path,
                    select_new_prim=False,
                )

                omni.kit.commands.execute(
                'BindMaterial',
                prim_path=prim.GetPath(),
                material_path=floor_material_prim_path,
                strength=pxr.UsdShade.Tokens.strongerThanDescendants
                )

        for prim in wall_parent.GetChildren():
            if prim is None:
                raise Exception("no house in scene!")
                
            carb.log_info("changing material at path: " + prim.GetPath().pathString)

            if wall_material_prim_path:
                omni.kit.commands.execute(
                    "CreateMdlMaterialPrim",
                    mtl_url=wall_mtl_url,
                    mtl_name=wall_mtl_name,
                    mtl_path=wall_material_prim_path,
                    select_new_prim=False,
                )

                omni.kit.commands.execute(
                'BindMaterial',
                prim_path=prim.GetPath(),
                material_path=wall_material_prim_path,
                strength=pxr.UsdShade.Tokens.strongerThanDescendants
                )

    def randomize_material(self):
        """
        randomize material for mobility
        """
        self.setup_material_helper()

        # print("house material_dict: ", self.material_dict)
        # print(os.getcwd())

        # if selected, update selection materials
        prim_paths = omni.usd.get_context().get_selection().get_selected_prim_paths()
        if prim_paths and len(prim_paths) > 0:
            pass
        else:
            # find target object
            target_obj_id = str(self.task_json["object_id"])
            obj_prim = None
            self.stage = omni.usd.get_context().get_stage()
            game_parent = self.stage.GetPrimAtPath("/World/game")
            for prim in game_parent.GetChildren():
                # if no materials
                if target_obj_id in prim.GetPath().pathString:
                    obj_prim = prim
                    break
            
            # print("obj_path_string", obj_prim.GetPath().pathString)


            if  len(self.random_info["materials"]) == 0:
                material_list = [x for v in self.material_dict.values() for x in v]
                mat_urls = random.sample(material_list, 10) # random sample ten materials 80% train 20% test
                self.random_info["materials"] = {"train":mat_urls[:8], "test":mat_urls[8:]}
                # self.save_asset_info()
            
            # if has materials, load train material type
            self.material_rnd = (1 + self.material_rnd) % len(self.random_info["materials"]["train"])

            mtl_url = self.random_info["materials"]["train"][self.material_rnd] #random.choice(self.random_info["materials"]["train"])
            mtl_name = mtl_url.split("/")[-1][:-4]

            if obj_prim is None:
                raise Exception(f"must load mobility first (object id){target_obj_id}")

            carb.log_info("changing material at path: " + obj_prim.GetPath().pathString)

            # change mtl
            new_looks_path, material_prim_path = get_material_prim_path(mtl_name)
            if new_looks_path:
                omni.kit.commands.execute(
                    "CreatePrim", prim_path=new_looks_path, prim_type="Scope", select_new_prim=False
                )
            
            if material_prim_path:
                omni.kit.commands.execute(
                    "CreateMdlMaterialPrim",
                    mtl_url=mtl_url,
                    mtl_name=mtl_name,
                    mtl_path=material_prim_path,
                    select_new_prim=False,
                )

                omni.kit.commands.execute(
                'BindMaterial',
                prim_path=obj_prim.GetPath(),
                material_path=material_prim_path,
                strength=pxr.UsdShade.Tokens.strongerThanDescendants
                )
        
        # mat_type = random.choice(list(self.material_dict.keys()))
        # mtl_url = random.choice(self.material_dict[mat_type])
        # mtl_name = mtl_url.split("/")[-1][:-4]

        # # mtl_url = "http://localhost:8080/omniverse://127.0.0.1/NVIDIA/Materials/Base/Architecture/Ceiling_Tiles.mdl"
        # # mtl_name = "Ceiling_Tiles"

        # new_looks_path, material_prim_path = get_material_prim_path(mtl_name)
        # if new_looks_path:
        #     omni.kit.commands.execute(
        #         "CreatePrim", prim_path=new_looks_path, prim_type="Scope", select_new_prim=False
        #     )
        
        # if material_prim_path:
        #     omni.kit.commands.execute(
        #         "CreateMdlMaterialPrim",
        #         mtl_url=mtl_url,
        #         mtl_name=mtl_name,
        #         mtl_path=material_prim_path,
        #         select_new_prim=False,
        #     )

        #     for prim_path in prim_paths:
        #         omni.kit.commands.execute(
        #         'BindMaterial',
        #         prim_path=prim_path,
        #         material_path=material_prim_path,
        #         strength=pxr.UsdShade.Tokens.strongerThanDescendants
        #         )
    
    def record_game_offset(self):
        # record game xform position and rotation
        self.stage = omni.usd.get_context().get_stage()
        game_prim = self.stage.GetPrimAtPath("/World/game") #pxr.UsdGeom.Xform.Get(self.stage, "/World/game")
        if game_prim:
            quad = game_prim.GetAttribute("xformOp:orient").Get()
            translate = game_prim.GetAttribute("xformOp:translate").Get()
            # print("game_prim", game_prim, eval(str(quad)))
            quad = eval(str(quad))

            layout_offset = {
                "translate": [translate[0], translate[1], translate[2]],
                "orient": [quad[0], quad[1], quad[2], quad[3]], 
                "scale": [1.0, 1.0, 1.0],
            }

            # check if currect layout offset is already recorded
            layout_offset_already_recorded = False
            #if "layout_offsets" in self.random_info["locations"]:
            for offset in self.random_info["locations"]:
                #if offset == layout_offset:
                print("offset", offset)
                if offset["translate"] == layout_offset["translate"] and \
                    offset["orient"] == layout_offset["orient"] and \
                        offset["scale"] == layout_offset["scale"]:
                    layout_offset_already_recorded = True
                    break
            
            # if not in record, add offset record
            if not layout_offset_already_recorded:                
                self.random_info["locations"].append(layout_offset)

            print("New game offset recorded at: ", layout_offset)

    def record_randomization(self):
        with open(self.task_json_path, "w") as f:
            json.dump(self.task_json, f, indent=4)

    
    def randomize_sky(self, sky_type:str = None, url= "http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Skies/Dynamic/"):
        """
        Add sky to the environment
        """
        # return
     
        # FIXME: not compatible with new version
        self.stage = omni.usd.get_context().get_stage()

        ENVIRONMENT_ROOT = "/Environment"
        sky_prim_path = f"{ENVIRONMENT_ROOT}/sky"

        # disable light
        # light_prim_path = "/World/defaultLight"
        # light_prim = self.stage.GetPrimAtPath(light_prim_path)
        # if light_prim:
        #     light_prim.GetAttribute('visibility').Set('invisible')

        if sky_type:
            sky_name = f"{sky_type}Sky" if not sky_type == "Overcast" else "Overcast"
        else:
            sky_list = ["ClearSky","CloudySky","Overcast","NightSky"]
            sky_name = random.choice(sky_list)
            
        sky_url = f"{url}{sky_name}.usd"

        # if found existing env, return
        sky_prim = self.stage.GetPrimAtPath(sky_prim_path)
        if sky_prim:
            carb.log_warn("Sky already in the env")
            sky_prim.GetReferences().ClearReferences()
        else:
            sky_prim = self.stage.DefinePrim(sky_prim_path, "Xform")

        if len(sky_type) == 0:
            # invalid sky type:
            return
        
        sky_prim.GetReferences().AddReference(sky_url)
        rot = pxr.Gf.Vec3d(0, 0, 0)
        properties = sky_prim.GetPropertyNames()
        if "xformOp:rotateXYZ" in properties:
            rotation = sky_prim.GetAttribute("xformOp:rotateXYZ")
            rotation.Set(rot)
        elif "xformOp:rotateZYX" in properties:
            rotation = sky_prim.GetAttribute("xformOp:rotateZYX")
            rotation.Set(rot)
        elif "xformOp:transform" in properties:
            carb.log_info("Object missing rotation op. Adding it.")
            xform = pxr.UsdGeom.Xformable(sky_prim)
            xform_op = xform.AddXformOp(pxr.UsdGeom.XformOp.TypeRotateXYZ, pxr.UsdGeom.XformOp.PrecisionDouble, "")
            rotate = Gf.Vec3d(rot[0], rot[1], rot[2])
            xform_op.Set(rotate)

            # if IS_IN_ISAAC_SIM:
            #     from omni.isaac.core.utils.stage import add_reference_to_stage
            #     add_reference_to_stage(sky_url ,sky_prim_path)
            # else:
            #     omni.kit.commands.execute("CreateUsdSkyPrimCommand", sky_url=sky_url, sky_path=sky_prim_path)

            # too light, lower intensity to pretect eyes
            # 
            # domelight_prim = self.stage.GetPrimAtPath("/Environment/sky/DomeLight")
            # domelight_prim.GetAttribute("intensity").Set(0)


        