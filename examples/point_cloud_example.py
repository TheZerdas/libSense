import open3d as o3d

# define path, RGBimg_name, Depthimg_name, point_cloud_name,
# intrinsic, voxel_size, RGB-D method
# if you don't define, it will choose its own in current directory
path = (r"C:\Users\Python\....")
RGBname = "sensergb.jpg"
Depthname = "senseD.png"
point_cloud_name = "point_cloud.pcd"
intrinsic = "intrinsic.json"
voxel_size = 0.00006
# rgbd = "color_and_depth"
rgbd = "redwood"
# rgbd = "sun"
# rgbd = "tum"

import libSense.process as lib
cloud = lib.process(path, RGBname,Depthname, voxel_size=voxel_size,rgbd=rgbd)

pcd = cloud.get_point_cloud()
o3d.visualization.draw_geometries([pcd])
