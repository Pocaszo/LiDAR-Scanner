# config_loader.py
import os
from apps.config.config import Configuration  # adjust import path if needed

# 🟦 Base Path
try:
    CWD_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir, os.pardir))
except NameError:
    CWD_PATH = os.getcwd()

# 🟦 Config Load
CONFIG_PATH = os.path.join(CWD_PATH, 'config', 'lidar_config.json')

config = Configuration()
config.read_config(CONFIG_PATH)

__all__ = ["config"]
