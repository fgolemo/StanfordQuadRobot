from datetime import datetime
import numpy as np
import matplotlib.pyplot as plt
import pickle
import os

# base_path = os.path.expanduser("~/dev/pupper-recordings/walk-forward/")
base_path = os.path.expanduser("~/dev/pupper-recordings/walk-forward/")

with open(base_path + "1600636149.5109496.pkl", "rb") as handle:
    data = pickle.load(handle)

print(f"recording len {len(data) / 100}s")
# for key, val in data.items():
#     print(datetime.fromtimestamp(float(key)))

foot_positions = np.array([x["feet"] for _, x in data.items()])
joint_positions = np.array([x["joints"] for _, x in data.items()])

print(foot_positions.shape)
print(joint_positions.shape)

coord = 0 # forward-backward
coord = 1 # sideways
coord = 2 # up-down
for i in range(4):
    plt.plot(np.arange(len(foot_positions[:, coord, i])), foot_positions[:, coord, i], label=f"Foot {i+1}")
plt.legend()
plt.show()

# coord = 0
# for i in range(4):
#     plt.plot(np.arange(len(joint_positions[:, coord, i])), joint_positions[:, coord, i], label=f"Joint {i+1}")
# plt.legend()
# plt.show()
