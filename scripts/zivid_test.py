from netCDF4 import Dataset
from matplotlib import pyplot as plt

f = Dataset(filename='/home/xintong/Documents/PyProjects/ICP_test/src/Zivid_project/point_clouds/01.zdf',
            mode='r')

# Extract the point cloud
pc = f['data']['pointcloud'][:,:,:]

# Extract the RGB image
image = f['data']['rgba_image'][:,:,:]

# Plot the RGB image
plt.imshow(image)

# Plot the Z component
plt.imshow(pc[:,:,2], vmin=0, vmax=1500)

# Print camera settings
print(f['metadata']['Settings']['Camera'])

# Print measurement settings
print(f['metadata']['Settings']['Measurement'])

# Print state information (e.g. temperature)
print(f['metadata']['State'])

# Close the ZDF file
f.close()
