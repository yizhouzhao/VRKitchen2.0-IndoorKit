# task labeling suggestion
from logging import root
from omni import ui
import os
import json
import carb
from ..param import DATA_PATH_NEW, TASK_TYPES, ANNOTATORS

def generate_suggestion_text_from_list(id_list):
    if len(id_list) == 0:
        return "no suggestion"
    return ",".join([str(_) for _ in id_list])

class AutoSuggest():
    def __init__(self) -> None:
        pass

    def read_ui(self):
        self.task_type_index = self.suggest_task_type_ui.model.get_item_value_model().get_value_as_int()
        self.task_type = TASK_TYPES[self.task_type_index - 1]
        self.task_id = self.suggest_task_id_ui.model.get_value_as_int()
        self.robot_id = self.suggest_robot_id_ui.model.get_value_as_int()
        self.mission_id = self.suggest_mission_id_ui.model.get_value_as_int()
        self.house_id = self.suggest_house_id_ui.model.get_value_as_int()
        self.anchor_id = self.suggest_anchor_id_ui.model.get_value_as_int()
        self.annotator_index = self.annotator_ui.model.get_item_value_model().get_value_as_int()
        self.annotator = ANNOTATORS[self.annotator_index]

    def reset_ui(self):
        self.suggest_task_type_ui.model.get_item_value_model().set_value(0)
        self.suggest_task_id_ui.model.set_value(-1)
        self.suggest_robot_id_ui.model.set_value(-1)
        self.suggest_mission_id_ui.model.set_value(-1)
        self.suggest_house_id_ui.model.set_value(-1)
        self.suggest_anchor_id_ui.model.set_value(-1)
        
        self.suggest_task_id_text_ui.model.set_value("")
        self.suggest_robot_id_text_ui.model.set_value("")
        self.suggest_mission_id_text_ui.model.set_value("")
        self.suggest_anchor_id_text_ui.model.set_value("")
        self.suggest_house_id_text_ui.model.set_value("")
        self.info_ui.model.set_value("")
    
    def suggest_trial_num(self):
        from ..param import SAVE_ROOT
        root_dir = '-'.join([self.task_type, str(self.task_id), str(self.robot_id), str(self.mission_id), str(self.house_id), \
            str(self.anchor_id) ])
        folders = os.listdir(SAVE_ROOT)
        folders = [folder  for folder in folders if folder.startswith(root_dir)]
        return len(folders)

        
    def suggest_task(self):
        self.read_ui()
        task_ids = os.listdir(os.path.join(DATA_PATH_NEW, self.annotator, "task", self.task_type))
        task_ids.sort(key=lambda x: int(x))
        self.suggest_task_id_text_ui.model.set_value(generate_suggestion_text_from_list(task_ids))

    def suggest_robot(self):
        self.read_ui()
        robot_file = os.path.join(DATA_PATH_NEW, self.annotator, "task", self.task_type, str(self.task_id), "robots.json")
        
        if os.path.exists(robot_file):
            robot_ids = list(json.load(open(robot_file)).keys())
        else:
            carb.log_warn(f"No robots found for task {self.task_type}: {self.task_id}")
            robot_ids = []

        # print(robot_ids)
        self.suggest_robot_id_text_ui.model.set_value(generate_suggestion_text_from_list(robot_ids))

    def suggest_anchor_id(self):
        self.read_ui()
        house_folder = os.path.join(DATA_PATH_NEW, self.annotator, "house")
        house_folders = os.listdir(house_folder)
        keys = []
        # folder: 0, 1, 2 etc...
        display = []
        for folder in house_folders:
            path = str(os.path.join(house_folder, folder, "anchor.json" ))
            if os.path.exists(path):
                with open(path) as f:
                    data = json.load(f)
                keys.extend(list(data.keys()))
                for name in keys:
                    tmp = name.split()
                    assert (len(tmp) == 4)
                    task_type = tmp[0]
                    task_id = tmp[1]
                    robot_id = tmp[2]
                    anchor_id = tmp[3]

                    if task_type == self.task_type and str(task_id) == str(self.task_id) and str(robot_id) == str(self.robot_id):
                        display.append(anchor_id)
        self.suggest_anchor_id_text_ui.model.set_value(generate_suggestion_text_from_list(display))

    def suggest_houseID(self):
        self.read_ui()
  
        house_folder = os.path.join(DATA_PATH_NEW, self.annotator, "house")
        house_folders = os.listdir(house_folder)
        keys = []
        # folder: 0, 1, 2 etc...
        display = []
        for folder in house_folders:
            path = str(os.path.join(house_folder, folder, "anchor.json" ))
            if os.path.exists(path):
                with open(path) as f:
                    data = json.load(f)
                keys.extend(list(data.keys()))
                for name in keys:
                    tmp = name.split()
                    assert (len(tmp) == 4)
                    task_type = tmp[0]
                    task_id = tmp[1]
                    robot_id = tmp[2]
                    anchor_id = tmp[3]

                    if task_type == self.task_type and str(task_id) == str(self.task_id) and str(robot_id) == str(self.robot_id):
                        display.append(folder)



        self.suggest_house_id_text_ui.model.set_value(generate_suggestion_text_from_list(display))
    
    def suggest_mission(self):
        self.read_ui()
        mission_file = os.path.join(DATA_PATH_NEW, self.annotator, "task", self.task_type, str(self.task_id), "missions.json")
        
        mission_ids = []
        if os.path.exists(mission_file):
            mission_info = json.load(open(mission_file))

            # identifier_prefix = self.task_type + " " + str(self.task_id) + " " + str(self.robot_id)
            identifier_prefix = self.task_type + " " + str(self.task_id) #+ " " + str(self.robot_id)
            for key in mission_info:
                if key.startswith(identifier_prefix):
                    mission_ids.append(key.split()[-1])
        else:
            carb.log_warn(f"No mission found for task {self.task_type}: {self.task_id}")
       
        self.suggest_mission_id_text_ui.model.set_value(generate_suggestion_text_from_list(mission_ids))
    
    def suggest_goal(self):
        self.read_ui()
        task_folder = os.path.join(DATA_PATH_NEW,  self.annotator, "task", self.task_type, str(self.task_id))
        if not os.path.exists(task_folder):
            carb.log_warn(f"Task folder not exist at {task_folder}")
            self.info_ui.model.set_value("Please add mission.")
        
        mission_file_path = os.path.join(task_folder, "missions.json")
        if os.path.exists(mission_file_path):
            missions = json.load(open(mission_file_path))
            carb.log_info(f"Loading missions.json at path {mission_file_path}")

           
            mission_identifier_prefix = self.task_type + " " + str(self.task_id) + " "
            mission_identifier_suffix = str(self.mission_id)

            for key, value in missions.items():
                if key.startswith(mission_identifier_prefix) and key.endswith(mission_identifier_suffix):
                    current_task = missions[key]
                    self.info_ui.model.set_value(json.dumps(current_task["goal"], indent = 2))
        else:
            self.info_ui.model.set_value("Please add mission.")




    
    