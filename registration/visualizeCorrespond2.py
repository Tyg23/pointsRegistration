import open3d as o3d
import numpy as np
import copy


def draw_registration_result(source, target):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1,0, 0]) #red
    target_temp.paint_uniform_color([0,0, 1]) #blue
    o3d.visualization.draw_geometries([source_temp, target_temp])


source = o3d.io.read_point_cloud("../pcdtarget2.txt", format='xyz')
target = o3d.io.read_point_cloud("../pcdsource2.txt", format='xyz')
xyz_source = np.asarray(source.points)
xyz_target = np.asarray(target.points)
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

o3d.visualization.draw_geometries([source,target,line_set])