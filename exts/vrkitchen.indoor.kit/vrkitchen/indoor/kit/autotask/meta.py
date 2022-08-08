import json
from pathlib import Path
import os

auto_folder = str(Path(__file__).parent.resolve()).replace("\\", "/")

# print("auto_folder", auto_folder)


AUTOTASK_META = json.load(open(os.path.join(auto_folder,"configs.json"))) 