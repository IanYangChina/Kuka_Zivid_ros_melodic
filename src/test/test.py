import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

script_dir = os.path.dirname(__file__)
file = os.path.join('/home/xintong/Downloads/', 'pos.npy')
pos = np.load(file)
plt.plot(pos[:, 0])
plt.show()
# exit()

positions = np.zeros(shape=(pos.shape[0], 3))
for n in range(pos.shape[0]):
    positions[n, 1] = pos[n, 0]
    positions[n, 2] = pos[n, 1]

delta_p = np.diff(positions, axis=0)
plt.plot(delta_p)
plt.show()
np.save(os.path.join(script_dir, '..', 'grasping_demo', 'src', 'delta_p.npy'), delta_p)

rots = np.zeros(shape=(pos.shape[0], 3))
for n in range(pos.shape[0]):
    r = R.from_quat([pos[n, 4], pos[n, 5], pos[n, 6], pos[n, 3]])
    rots[n, :] = r.as_euler('xyz', degrees=True)

rots[:, 0] = rots[:, 2]
rots[:, 2] *= 0
plt.plot(rots)
plt.show()
rot_diff = np.diff(rots, axis=0)
rot_diff[:, 1] *= 0
rot_diff[:, 2] *= 0
plt.plot(rot_diff)
plt.show()
np.save(os.path.join(script_dir, '..', 'grasping_demo', 'src', 'rot_diff.npy'), rot_diff)
