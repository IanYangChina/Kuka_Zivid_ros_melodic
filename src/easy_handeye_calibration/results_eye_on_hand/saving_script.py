import os
import numpy as np

script_path = os.path.dirname(os.path.realpath(__file__))
raw_transform_ee_to_cam = np.array(
    [[0.08013885, -0.93715333, -0.33959004, 0.173],
     [0.98541201, 0.02317079, 0.16860094, -0.065],
     [-0.15013637, -0.34814759, 0.92533903, 0.015],
     [0., 0., 0., 1.]]
)
raw_transform_cam_to_ee = np.linalg.inv(raw_transform_ee_to_cam)
np.save(os.path.join(script_path, 'transform_ee_to_cam.npy'), raw_transform_ee_to_cam)
np.save(os.path.join(script_path, 'transform_cam_to_ee.npy'), raw_transform_cam_to_ee)
