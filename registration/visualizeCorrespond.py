import open3d as o3d
import numpy as np
import copy


def draw_registration_result(source, target):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1,0, 0]) #red
    target_temp.paint_uniform_color([0,0, 1]) #blue
    o3d.visualization.draw_geometries([source_temp, target_temp])


source = o3d.io.read_point_cloud("./data3/Lowersource.txt", format='xyz')
target = o3d.io.read_point_cloud("./data3/Lowertarget.txt", format='xyz')
source2 = o3d.io.read_point_cloud("./data3/Uppersource.txt", format='xyz')
target2 = o3d.io.read_point_cloud("./data3/Uppertarget.txt", format='xyz')
# vessel1 = o3d.io.read_point_cloud("./data3/UpperResult.txt", format='xyz')
# vessel2 = o3d.io.read_point_cloud("./data3/LowerResult.txt", format='xyz')
vessel2=o3d.io.read_point_cloud("./data3/whole_vessel_trajectory.txt", format='xyz')
vessel3 = o3d.io.read_point_cloud("./data3/vessel_trajectory.txt", format='xyz')
arm = o3d.io.read_point_cloud("./data3/ARM.xyz", format='xyz')
arm2 = o3d.io.read_point_cloud("./data3/segedArm.txt", format='xyz')
xyz_source = np.asarray(source.points)
xyz_target = np.asarray(target.points)
xyz_source2 = np.asarray(source2.points)
xyz_target2 = np.asarray(target2.points)
xyz_arm = np.asarray(arm.points)
points=np.vstack((xyz_source,xyz_target))
lines=[]
n=int(xyz_source.size/3)
print(n)
for i in range(0,n,30):
    lines.append([i,i+n])
colors = [[0, 1, 0] for i in range(len(lines))] #yellow
line_set = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(points),
    lines=o3d.utility.Vector2iVector(lines),
)
line_set.colors = o3d.utility.Vector3dVector(colors)
source.paint_uniform_color([0.6, 0.6, 0])  # red
target.paint_uniform_color([0, 0, 1])  # blue

points2=np.vstack((xyz_source2,xyz_target2))
lines2=[]
n2=int(xyz_source2.size/3)
print(n2)
for i in range(0,n2,30):
    lines2.append([i,i+n2])
colors2 = [[0, 1, 0] for i in range(len(lines2))] #yellow
line_set2 = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(points2),
    lines=o3d.utility.Vector2iVector(lines2),
)
line_set2.colors = o3d.utility.Vector3dVector(colors2)
source2.paint_uniform_color([0.6, 0.6, 0])  # red
target2.paint_uniform_color([0, 0, 1])  # blue

# vessel1.paint_uniform_color([1, 0, 0])  # red
vessel2.paint_uniform_color([1, 0, 0])  # red
vessel3.paint_uniform_color([1, 0, 0])  # red

arm2.paint_uniform_color([0, 0, 1])  # red

# arm.paint_uniform_color([1, 1, 0])  # yellow
# o3d.visualization.draw_geometries([source,target,line_set])
# textured_mesh= o3d.io.read_triangle_mesh('./data2/ARM.obj')
# print(textured_mesh)
# textured_mesh.compute_vertex_normals()
# o3d.visualization.draw_geometries([textured_mesh])

# obj顶点显示
# pcobj = o3d.geometry.PointCloud()
# pcobj.points = o3d.utility.Vector3dVector(textured_mesh.vertices)
# pcobj.paint_uniform_color([0.6, 0.6, 0.1])
# o3d.visualization.draw_geometries([source,target,line_set,pcobj,textured_mesh])
o3d.visualization.draw_geometries([source,target,source2,target2,line_set,line_set2,vessel2,vessel3,arm2])
# o3d.visualization.draw_geometries([xyz_arm])
# draw_registration_result(source, target)
# source = o3d.io.read_point_cloud("./data/segmented_cloud_human.pcd")
# o3d.visualization.draw_geometries([source])