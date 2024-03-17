import open3d as o3d
from open3d.data import BunnyMesh
from numpy import load

xyz = load('/home/ben/thyroid_ultrasound_data/testing_and_validation/volume_data/VolumeData_1710294534_632789850.npy')
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(xyz)

radii = [0.005, 0.01, 0.02, 0.04]

# dataset = o3d.data.BunnyMesh()
# mesh = o3d.io.read_triangle_mesh(dataset.path)
# pcd = mesh.sample_points_poisson_disk(750)
o3d.visualization.draw_geometries([pcd])
alpha = 3.5
print(f"alpha={alpha:.3f}")
radii = [0.005, 0.01, 0.02, 0.04]
rec_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd=pcd, radii=o3d.utility.DoubleVector(radii))
o3d.visualization.draw_geometries([pcd, rec_mesh])
