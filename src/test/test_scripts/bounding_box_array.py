import os
import numpy as np
import open3d as o3d

script_path = os.path.dirname(os.path.realpath(__file__))
workspace_bounding_box_array = np.array(
    [[-0.35, 0.2, 0.2],
     [-0.95, 0.2, 0.2],
     [-0.35, -0.3, 0.2],
     [-0.95, -0.3, 0.2],
     [-0.35, 0.2, 0.045],
     [-0.95, 0.2, 0.045],
     [-0.35, -0.3, 0.045],
     [-0.95, -0.3, 0.045]]
)
np.save(os.path.join(script_path, 'transformation_matrices', 'workspace_bounding_box_array_in_base.npy'),
        workspace_bounding_box_array)
print(workspace_bounding_box_array)
