import omni
import numpy as np 

try:
    import pandas as pd
except:
    omni.kit.pipapi.install("pandas")
    import pandas as pd

GOODLE_SHEET_INFO = {
    "close_cabinet": "187VN5J70tEH6ByemAs60FRA2uxE5UmtMr2rBZ0DCOAs",
    "close_door": "1Lm-nqYdeUfjGZc2WyqJCG5JcI1z5zDhfeoxZiUX7VKE",
    "close_drawer": "1OMmuQNKcvbc-CQm67CQbQSmiQGRMVXtNYYXgTsNg9NE",
    "open_cabinet": "1SWXaK5v701wMklIMu4MTgh8Wes5WS9bd_YTrH9-DPdw",
    "open_drawer": "1DHYxbRRs0i11rEmDKJ7XK4H0UTTct2QpPTpIPkHnImU",
    "pickup_object": "1mq7qCTsJWKnr1-MWA7kzOehZM6fw-o8iHpqKAS6PM44",
    "pour_water": "1mS1HUljpu2tZCfiHNvHc2FfrsvGFzwyXRm6pqj3uzZU",
    "reorient_object": "1VyoSXjUxp5ef2RPGRxovIv3SA5rr-gm66sjABegqcwM",
    "transfer_water":  "1fKLFHfF3LsYIWlheqQwGHIf6Bpn05BnT-AQheANyO6o",
    "tap_water": "1kgXT6baclDuvyCe4ijJgrR1xTDbkZggxP7d5gQpWR8w",
    "open_door": "1fKp1vzDMeoR0lPspqtVZTaHdNhCyXdJ6SN2EnIjQ6CA",
}

# for key in GOODLE_SHEET_INFO:
#     sheet_id = GOODLE_SHEET_INFO[key]
#     test = pd.read_csv(f"https://docs.google.com/spreadsheets/d/{sheet_id}/export?format=csv")
#     print(test.head())


class AutoLabeler():
    def __init__(self, task_type) -> None:
        # load task
        self.task_type = task_type
        
        self.cache = {}
        # for task_type_cache in GOODLE_SHEET_INFO.keys():
        #     cache_id = GOODLE_SHEET_INFO[task_type_cache]
        #     try:
        #         self.cache[task_type_cache] = pd.read_csv(f"https://docs.google.com/spreadsheets/d/{cache_id}/export?format=csv")
        #     except:
        #         print("service not available: ", task_type_cache)
        
        # load data
        if self.task_type:
            sheet_id = GOODLE_SHEET_INFO[self.task_type]
            self.data = pd.read_csv(f"https://docs.google.com/spreadsheets/d/{sheet_id}/export?format=csv")
            self.cache[task_type] = self.data
        
        # load id
        self.current_id = -1
    
    def set_task_type(self, task_type):
        if task_type not in self.cache:
            cache_id = GOODLE_SHEET_INFO[task_type]
            try:
                self.cache[task_type] = pd.read_csv(f"https://docs.google.com/spreadsheets/d/{cache_id}/export?format=csv")
            except:
                print("service not available: ", task_type)
        self.data = self.cache[task_type]
    
    def set_id(self, id):
        """
        set current id
        """
        self.current_id = id

    def find_row_num(self, task_id, robot_id, mission_id, house_id, trial_id):
        
        cond = np.where(  (self.data['task_id'] == int(task_id)) & (self.data['robot_id'] == int(robot_id))  &
         (self.data['mission_id'] == int(mission_id))  &  (self.data['house_id'] == int(house_id))  &  (self.data['trial_id'] == int(trial_id)) 
          ) 

        try:
            return int(cond[0])+2
        except:
            return -1
        

    def load_row(self):
        """
        Load task information from row_id
        """
        assert self.current_id >= 0
        if self.current_id >= len(self.data):
            raise Exception(f"Note: current labeling is done {self.task_type}: {self.current_id} / {len(self.data)}")

        id = self.current_id
        task_id = self.data["task_id"][id]
        robot_id = self.data["robot_id"][id]
        mission_id = self.data["mission_id"][id]
        house_id = self.data["house_id"][id]
        trial_id = self.data["trial_id"][id]

        return int(task_id), int(robot_id), int(mission_id), int(house_id), int(trial_id)
    
    def next(self):
        """
        find next id
        """
        if self.current_id >= 0:
            self.current_id += 1
        else:
            """
            find current labeling index
            """
            for i in range(len(self.data)):
                if pd.isnull(self.data['progress'][i]):
                    self.current_id = i
                    return 
        
        

