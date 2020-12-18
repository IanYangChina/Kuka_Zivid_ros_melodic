import numpy as np
from scipy.spatial.transform import Rotation
# qw: 0.035
# qx: 0.003
# qy: -0.995
# qz: 0.086
print("non_rectified camera angle", Rotation.from_quat([0.003, -0.995, 0.086, 0.035]).as_euler('zxy', degrees=True))
# x: -0.526
# y: 0.162
# z: 1.090
x1 = np.array([-0.526, 0.162, 1.090])

# qw: 0.085
# qx: -0.030
# qy: -0.991
# qz: 0.093
print("rectified camera angle", Rotation.from_quat([-0.030, -0.991, 0.093, 0.085]).as_euler('zxy', degrees=True))
# x: -0.456
# y: 0.150
# z: 1.060
x2 = np.array([-0.456, 0.150, 1.060])

print(np.sqrt(np.sum(np.power((x1-x2), 2))))

# results:
# non_rectified camera angle [ 2.91182958e-03  9.87974145e+00 -1.75970551e+02]
# rectified camera angle [   4.39312877   10.33664508 -169.79770538]
# 0.07709734106958555
