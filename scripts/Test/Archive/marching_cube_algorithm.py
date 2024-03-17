from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from skimage import measure

# In my case, I have position of voxels in a MRI volume
points = np.array([[-0.025,  0.015, -0.04 ],
                   [-0.02 ,  0.015, -0.04 ],
                   [-0.03 ,  0.015, -0.035],
                   [-0.025,  0.015, -0.035],
                   [-0.02 ,  0.015, -0.035],
                   [-0.015,  0.015, -0.035],
                   [-0.025,  0.01 , -0.03 ],
                   [-0.025,  0.015, -0.03 ],
                   [-0.02 ,  0.015, -0.03 ],
                   [-0.015,  0.015, -0.03 ]])

# And I have a transformation matrix that took the voxels indices
# to these 3D coordinates
trans_mat = np.array([[ 0.005,  0.   ,  0.   , -0.06 ],
                      [ 0.   ,  0.005,  0.   , -0.075],
                      [ 0.   ,  0.   ,  0.005, -0.075],
                      [ 0.   ,  0.   ,  0.   ,  1.   ]])

# The shape of my MRI voxel volume
volume_shape = (25, 32, 26)

# From 3D points to voxel indices
points_4d = np.hstack((points, np.ones((points.shape[0], 1))))
trans_points = np.linalg.inv(trans_mat)  @  points_4d.T

voxels = np.zeros(volume_shape)
for ix, iy, iz in np.round(trans_points[:3, :]).T.astype(int):
    voxels[ix, iy, iz] = 1

vertices, faces = measure.marching_cubes(voxels, spacing=(1, 1, 1))[:2]

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot_trisurf(vertices[:, 0], vertices[:,1], faces, vertices[:, 2],
                cmap='Spectral', lw=1)

plt.show()