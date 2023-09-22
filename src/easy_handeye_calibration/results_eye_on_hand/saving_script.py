import os
import numpy as np
from scipy.spatial.transform import Rotation

script_path = os.path.dirname(os.path.realpath(__file__))
raw_transform_ee_to_cam = np.array(
    [[0.06321771, -0.9263225, -0.37138949, 0.196],
     [0.99119625, 0.01489889, 0.13155995, -0.041],
     [-0.11633365, -0.37643679, 0.91910926, 0.018],
     [0., 0., 0., 1.]]
)

euler = np.array([-21.5, 6.6, 87.5])
raw_transform_ee_to_cam[:-1, :-1] = Rotation.from_euler('xyz', euler, degrees=True).as_matrix()

raw_transform_cam_to_ee = np.linalg.inv(raw_transform_ee_to_cam)
np.save(os.path.join(script_path, 'transform_ee_to_cam.npy'), raw_transform_ee_to_cam)
np.save(os.path.join(script_path, 'transform_cam_to_ee.npy'), raw_transform_cam_to_ee)
