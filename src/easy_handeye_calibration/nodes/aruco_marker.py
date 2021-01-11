from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl

aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)

img = aruco.drawMarker(aruco_dict, 35, 100)
plt.imshow(img, cmap = mpl.cm.gray, interpolation = "nearest")
plt.show()
