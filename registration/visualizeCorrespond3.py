import open3d as o3d
import numpy as np
import copy


def draw_registration_result(source, target):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1,0, 0]) #red
    target_temp.paint_uniform_color([0,0, 1]) #blue
    o3d.visualization.draw_geometries([source_temp, target_temp])


source = o3d.io.read_point_cloud("../data2/vessel2.txt", format='xyz')
target = o3d.io.read_point_cloud("../data2/vessel.txt", format='xyz')
arm= o3d.io.read_point_cloud("../data2/ARM.xyz", format='xyz')
xyz_source = np.asarray(source.points)
xyz_target = np.asarray(target.points)
points=np.vstack((xyz_source,xyz_target))
lines=[]
n=int(xyz_source.size/3)
print(n)
for i in range(0,n,1):
    lines.append([i,i+n])
colors = [[0, 0, 1] for i in range(len(lines))] #yellow
line_set = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(points),
    lines=o3d.utility.Vector2iVector(lines),
)
line_set.colors = o3d.utility.Vector3dVector(colors)
target.paint_uniform_color([253/255,95/255,0])  # red
source.paint_uniform_color([221/255,10/255,53/255])  # blue
arm.paint_uniform_color([210/255,200/255,200/255])  # blue

# o3d.visualization.RenderOption.line_width=20
o3d.visualization.draw_geometries([arm,source,target,target,target,line_set])