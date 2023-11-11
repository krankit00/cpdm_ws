import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
import pandas as pd

pcd = o3d.io.read_point_cloud('~/Desktop/results/cpdm-test/dense/fused.ply')
# o3d.visualization.draw_geometries([pcd])
print(pcd)
arr = np.asarray(pcd.points)
# print(np.asarray(pcd.points))
z_min = arr[:,2].min()
z_max = arr[:,2].max()

Zmin = 3.89
Zmax = 4.2
d = 0.1
final_pointcloud_array = []
pointcloud_as_array = np.asarray(pcd.points)


for point in pointcloud_as_array:
    if  Zmin - d < point[2] < Zmax + d:
        final_pointcloud_array.append(point)
final_pointcloud = o3d.geometry.PointCloud()
final_pointcloud.points = o3d.utility.Vector3dVector(final_pointcloud_array)
# o3d.visualization.draw_geometries([final_pointcloud])

with o3d.utility.VerbosityContextManager(
    o3d.utility.VerbosityLevel.Debug) as cm:
    labels = np.array(
        final_pointcloud.cluster_dbscan(eps=0.05, min_points=4, print_progress=True))
    
max_label = labels.max()
print(f"point cloud has {max_label + 1} clusters")
colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
colors[labels < 0] = 0
final_pointcloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
# o3d.visualization.draw_geometries([final_pointcloud], mesh_show_wireframe=True, point_show_normal=True)

my_points = np.asarray(final_pointcloud.points).tolist()
df = pd.DataFrame(list(zip(my_points, labels.tolist())), columns=['points', 'label'])

new_df = df.groupby('label').agg(list).reset_index()[['label', 'points']]

label_to_points = new_df.set_index('label').to_dict()['points']

point_to_label = {}
for k, v in label_to_points.items():
    for aaa in v:
        aaa = [round(i, 2) for i in aaa]
        new_key = '_'.join(list(map(str, aaa)))
        point_to_label[new_key] = k

point_to_search = '1.31_3.98_4.28'
cluster_of_the_said_point = point_to_label[point_to_search]
other_points_of_same_cluster = label_to_points[cluster_of_the_said_point]
arr_2d = np.array(other_points_of_same_cluster)
x_min = arr_2d[:,0].min()
x_max = arr_2d[:,0].max()
y_min = arr_2d[:,1].min()
y_max = arr_2d[:,1].max()
# print(other_points_of_same_cluster)

points = np.array([
[x_min, y_min, z_min],
[x_min, y_min, z_max],
[x_min, y_max, z_min],
[x_min, y_max, z_max],
[x_max, y_min, z_min],
[x_max, y_min, z_max],
[x_max, y_max, z_min],
[x_max, y_max, z_max]
])

with open("./g_planner/input.txt", "w") as txt_file:
    for line in points:
        lx = list(map(str, line.tolist()))
        txt_file.write(" ".join(lx) + "\n") 

# demo_pc = o3d.geometry.PointCloud()
# demo_pc.points = o3d.utility.Vector3dVector(np.array(other_points_of_same_cluster))
# o3d.visualization.draw_geometries([demo_pc])