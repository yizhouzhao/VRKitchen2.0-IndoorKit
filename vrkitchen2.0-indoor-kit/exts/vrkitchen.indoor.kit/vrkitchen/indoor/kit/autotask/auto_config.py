# automatically generation configs meta data for task generation
import json
import copy

g_meta_json_path = "./configs.json"
# initail and target value pair for continous task
g_init_target_value_pair = [
    (0, 0.25), (0, 0.5), (0, 0.75), (0, 1),
    (0.25, 0.5), (0.25, 0.75), (0.25, 1), 
    (0.5, 0.75), (0.5, 1),
    (0.75, 1)   
]

g_mission_template =  {
        "size": 0, 
        "orient": [0, 0, 0.7071068, 0.7071068],
        "robot_offset": [-40, 0, 0],
        "robot_orient": [0.7071068, -0.7071068,0, 0],
        
        "task_type": "", 
        "task_id": "",
        "robot_id": "",
        "mission_id": "",
        "goal":{
            "description":"Open the door a little.",
            "condition": {
                "init_value": -1,
                "type": "rotation",
                "target": "",
                "joint":"",
                "target_value": 0
            }
        }
    }


 
def add_continuous_meta_open_mission(task_type, meta_json_path = g_meta_json_path):
    """
    add continous mission types for open task
    """
    # load json
    assert task_type in ["open_door", "open_drawer", "open_cabinet", "close_door", "pour_water", 
        "close_drawer", "close_cabinet", "transfer_water", "tap_water"]

    meta_json = json.load(open(meta_json_path))
    # if task_type not in meta_json:
    # clean 
    meta_json[task_type] = []
    task_missions  = meta_json[task_type]

    for init_value, target_value in g_init_target_value_pair:
        mission = copy.deepcopy(g_mission_template)
        goal = mission["goal"]
        condition = goal["condition"]
        
        if task_type == "open_door":
            #mission["robot_offset"] = [-40, 0, 0]
            mission["robot_offset"] = [50, 0, 0]
            mission["robot_orient"] = [0,0,0.7071068,0.7071068]

            goal["description"] = "Open the door"
            condition["type"] = "rotation"
            condition["init_value"] = init_value
            condition["target_value"] = target_value
        elif task_type == "close_door":
            mission["robot_offset"] = [70, 0, 0]
            mission["robot_orient"] = [0,0,0.7071068,0.7071068]
            goal["description"] = "close the door"
            condition["type"] = "rotation"
            condition["init_value"] = target_value
            condition["target_value"] = init_value
        elif task_type == "pour_water":
            # only pour half and empty
            if not (init_value, target_value) in [(0.5, 1), (0, 1)]:
                continue
            mission["robot_offset"] = [-30, 0, 0]
            goal["description"] = "Pour the liquid out of the contrainer."
            condition["type"] = "liquid"
            condition["init_value"] = target_value
            condition["target_value"] = init_value
            mission["size"] = 1.0
            mission["orient"] = [1, 0, 0, 0]
        elif task_type == "transfer_water":
            # only pour half and empty
            if not (init_value, target_value) in [(0, 0.25), (0, 0.5), (0, 0.75), (0, 1)]:
                continue
            mission["robot_offset"] = [-30, 0, 0]
            goal["description"] = "Pour the liquid into another contrainer."
            condition["type"] = "liquid"
            # condition["init_value"] = target_value
            condition["target_value"] = target_value
            mission["size"] = 1.0
            mission["orient"] = [1, 0, 0, 0]

        elif task_type == "close_drawer":
            condition["type"] = "linear"
            mission["robot_offset"] = [-70, 0, 0]
            goal["description"] = "close the drawer"
            condition["init_value"] = target_value
            condition["target_value"] = init_value
            mission["size"] = 70

        elif task_type == "open_drawer":
            condition["type"] = "linear"
            mission["robot_offset"] = [-50, 0, 0]
            goal["description"] = "Open the drawer"
            condition["init_value"] = init_value
            condition["target_value"] = target_value
            mission["size"] = 70
        
        elif task_type == "open_cabinet":
            condition["type"] = "rotation"
            mission["robot_offset"] = [-50, 0, 0]
            goal["description"] = "Open the cabinet"
            condition["init_value"] = init_value
            condition["target_value"] = target_value
            mission["size"] = 70

        elif task_type == "close_cabinet":
            condition["type"] = "rotation"
            mission["robot_offset"] = [-870, 0, 0]
            goal["description"] = "Close the cabinet"
            condition["init_value"] = target_value
            condition["target_value"] = init_value
            mission["size"] = 70

        elif task_type == "tap_water":
            # only pour half and empty
            if not (init_value, target_value) in [(0, 0.25), (0, 0.5), (0, 0.75), (0, 1)]:
                continue
            mission["robot_offset"] = [-30, 0, 0]
            goal["description"] = "Get tap water."
            condition["type"] = "liquid"
            condition["init_value"] = init_value
            condition["target_value"] = target_value
            mission["size"] = 20
            mission["orient"] = [0.7071068,-0.7071068,0,0]

        task_missions.append(mission)
    
    print("task_missions", task_missions)
    with open(meta_json_path, "w") as f: 
        json.dump(meta_json, f, indent = 4)
     

if __name__ == "__main__":
    print("genrating continous mission")  
    add_continuous_meta_open_mission("open_door") 
    

        




