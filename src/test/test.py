import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

script_dir = os.path.dirname(__file__)
file = os.path.join('/home/xintong/Downloads/pos_311_version/sample_step_10', 'pos3_10.npy')
pos = np.load(file)
plt.plot(pos[:, :3])
plt.show()

positions = np.zeros(shape=(pos.shape[0], 3))
for n in range(pos.shape[0]):
    positions[n, 1] = pos[n, 0]
    positions[n, 2] = pos[n, 1]

delta_p = np.diff(positions, axis=0) * 0.5
plt.plot(delta_p)
plt.show()
np.save(os.path.join(script_dir, '..', 'grasping_demo', 'src', 'delta_p_3.npy'), delta_p[:55, :])

rots = np.zeros(shape=(pos.shape[0], 3))
for n in range(pos.shape[0]):
    r = R.from_quat([pos[n, 4], pos[n, 5], pos[n, 6], pos[n, 3]])
    rots[n, :] = r.as_euler('xyz', degrees=True)
plt.plot(rots)
plt.show()
rots[:, 0] = rots[:, 2]
rots[:, 2] *= 0
plt.plot(rots)
plt.show()
rot_diff = np.diff(rots, axis=0)
rot_diff[:, 1] *= 0
rot_diff[:, 2] *= 0
plt.plot(rot_diff)
plt.show()
np.save(os.path.join(script_dir, '..', 'grasping_demo', 'src', 'delta_rot_3.npy'), rot_diff[:55, :])

