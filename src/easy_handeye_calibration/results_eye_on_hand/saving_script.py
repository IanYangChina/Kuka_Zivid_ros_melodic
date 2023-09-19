import os
import numpy as np

script_path = os.path.dirname(os.path.realpath(__file__))
raw_transform_ee_to_cam = np.array(
    [[0.06321771, -0.9263225, -0.37138949, 0.191],
     [0.99119625, 0.01489889, 0.13155995, -0.045],
     [-0.11633365, -0.37643679, 0.91910926, 0.013],
     [0., 0., 0., 1.]]
)
raw_transform_cam_to_ee = np.linalg.inv(raw_transform_ee_to_cam)
np.save(os.path.join(script_path, 'transform_ee_to_cam.npy'), raw_transform_ee_to_cam)
np.save(os.path.join(script_path, 'transform_cam_to_ee.npy'), raw_transform_cam_to_ee)
