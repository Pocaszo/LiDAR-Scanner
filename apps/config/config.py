# config.py
import json

class Configuration:
    def __init__(self):
        pass
    
    def read_config(self, config_file):
        # โหลด config
        try:
            with open(config_file, "r") as f:
                config = json.load(f)
        except:
            with open("lidar_config.json", "r") as f:
                config = json.load(f)

        self.config = config  # Save full config object for future use if needed
