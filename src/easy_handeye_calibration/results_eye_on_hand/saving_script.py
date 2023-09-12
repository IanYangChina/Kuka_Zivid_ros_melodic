import os
import numpy as np

script_path = os.path.dirname(os.path.realpath(__file__))
raw_transform_ee_to_cam = np.array(
    [[0.02055555, -0.9997352, 0.01034433, 0.119],
     [0.98554231, 0.02200199, 0.16799482, -0.07],
     [-0.16817793, 0.00674155, 0.9857336, 0.052],
     [0., 0., 0., 1.]]
)
raw_transform_cam_to_ee = np.linalg.inv(raw_transform_ee_to_cam)
np.save(os.path.join(script_path, 'transform_ee_to_cam.npy'), raw_transform_ee_to_cam)
np.save(os.path.join(script_path, 'transform_cam_to_ee.npy'), raw_transform_cam_to_ee)
