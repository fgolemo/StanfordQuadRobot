from datetime import datetime

import matplotlib.pyplot as plt
import pickle
import os

base_path = os.path.expanduser("~/dev/pupper-recordings/walk-forward/")

with open(base_path + "1600629019.2706409.pkl", "rb") as handle:
    data = pickle.load(handle)

print(f"recording len {len(data)/100}s")
for key, val in data.items():
    print(datetime.fromtimestamp(float(key)))

# Monday, June 29, 1970 9:56:49.563 AM
# Monday, June 29, 1970 9:56:49.563 AM
