import os
import numpy as np
import matplotlib.pyplot as plt


script_path = os.path.dirname(os.path.realpath(__file__))
data_path = os.path.join(script_path, '..', 'deformable_objects', 'pcd_to_mesh')

ind = 1
fig, axes = plt.subplots(1, 2, figsize=(2 * 2, 2))
plt.subplots_adjust(wspace=0.01)
axes[0].imshow(np.load(os.path.join(data_path, f'real_hm_{ind}.npy')),
               vmin=0, vmax=60, cmap='YlOrBr')
axes[0].get_xaxis().set_visible(False)
axes[0].get_yaxis().set_visible(False)
axes[0].set_title("real deformed shape")
axes[0].set_frame_on(False)
axes[1].imshow(np.load(os.path.join(data_path, f'sim_hm_{ind}.npy')),
               vmin=0, vmax=60, cmap='YlOrBr')
axes[1].get_xaxis().set_visible(False)
axes[1].get_yaxis().set_visible(False)
axes[1].set_title("sim deformed shape")
axes[1].set_frame_on(False)
plt.show()
