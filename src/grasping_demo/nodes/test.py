import os
import numpy as np
import open3d as o3d

script_path = os.path.dirname(os.path.realpath(__file__))
workspace_bounding_box_array = np.array(
    [[-0.2, 0.15, 0.2],
     [-0.65, 0.15, 0.2],
     [-0.2, -0.15, 0.2],
     [-0.65, -0.15, 0.2],
     [-0.2, 0.15, 0.03],
     [-0.65, 0.15, 0.03],
     [-0.2, -0.15, 0.03],
     [-0.65, -0.15, 0.03]]
)
np.save(os.path.join(script_path, '..', 'src', 'grasping_demo',
                     'transformation_matrices', 'workspace_bounding_box_array_in_base.npy'),
        workspace_bounding_box_array)
print(workspace_bounding_box_array)
workspace_bounding_box_array = o3d.utility.Vector3dVector(workspace_bounding_box_array.astype('float'))
workspace_bounding_box = o3d.geometry.OrientedBoundingBox.create_from_points(points=workspace_bounding_box_array)
workspace_bounding_box.color = (0, 1, 0)
