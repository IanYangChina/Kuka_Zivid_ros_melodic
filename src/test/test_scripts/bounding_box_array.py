import os
import numpy as np
import open3d as o3d

script_path = os.path.dirname(os.path.realpath(__file__))
workspace_bounding_box_array = np.array(
    [[-0.4, 0.1, 0.2],
     [-0.62, 0.1, 0.2],
     [-0.4, -0.08, 0.2],
     [-0.62, -0.08, 0.2],
     [-0.4, 0.1, -0.007],
     [-0.62, 0.1, -0.007],
     [-0.4, -0.08, -0.007],
     [-0.62, -0.08, -0.007]]
)
np.save(os.path.join(script_path, 'transformation_matrices', 'reconstruction_bounding_box_array_in_base.npy'),
        workspace_bounding_box_array)
print(workspace_bounding_box_array)
