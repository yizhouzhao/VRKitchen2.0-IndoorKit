try:
    import carb
except:
    pass
import os
from pathlib import Path

root = str(Path(__file__).parent.joinpath("../../../../../../").resolve())
ROOT = root
# print("root:", ROOT)
# ROOT = '/home/nikepupu/Desktop'
# In isaac sim or create
try:
    IS_IN_ISAAC_SIM = str(carb.settings.get_settings().get("/app/window/title")).startswith("Isaac Sim")
    IS_IN_CREAT = str(carb.settings.get_settings().get("/app/window/title")).startswith("Create")
    APP_VERION = str(carb.settings.get_settings().get("/app/version"))
except:
    pass

# root = '/home/yizhou/Research/' 

# root = '/home/vince/Documents/Research/' 
# ROOT = '/home/nikepupu/Desktop' if IS_IN_ISAAC_SIM else 'E:/researches' 

# Asset paths
ASSET_PATH =  ROOT + "/asset/"
SAPIEN_ASSET_PATH = ROOT  + "/asset/Sapien/"
HOUSE_INFO_PATH = ROOT +  "/asset/3DFront/" 
CUSTOM_ASSET_PATH = ROOT  + "/asset/Custom/" 
# STORAGE_ASSET_PATH =  ROOT + "/asset/sapien_parsed/StorageFurniture/"

# Data path
DATA_PATH = ROOT + "/data/"
DATA_PATH_NEW = ROOT + "/data_auto/"
ROBOT_PATH = ROOT + "/asset/robot/"

SAVE_ROOT = ROOT + '/data_record/'
ORIGINAL_IMAGES_FORLDER = "raw_images"
TRAJ_FOLDER = "trajectory"
DEPTH_IMAGES_FOLDER = "depth_images"
SEMANTIC_IMAGES_FOLDER = "semantic_images"

USE_ISO_SURFACE = False

#Annotator
ANNOTATORS = [
    "Steven",
    "Xiaofeng",
    "Yizhou",
    "AWS",
    "Ziheng",
]

# Task
TASK_TYPES = ["pickup_object","reorient_object", "transfer_water", "pour_water",
    "open_drawer","open_cabinet", "open_door"] # "put_object_into_box", 
    #"close_drawer", "close_cabinet", "close_door", "take_object_out_box"]

#Objects
OBJECT_TYPES = ["Bottle", "Box", "Door", "Faucet", "LightSwitch", "Microwave", "StorageFurniture"]

# Task objects
GAME_OBJ_NAMES = ["mobility", "switch", "SM_", "watercup", "fluid"]
CONTAINER_NAMES = ["box", "cup"]
OTHER_OBJ_NAMES = ["basin"]

# Physics
RIGIDBODY_OBJ_TYPES = ["Bottle", "SM_"]