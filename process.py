import usb.util
import open3d as o3d
import pyrealsense2 as rs
from numpy import *
import numpy as np
import cv2
import pcl
import os
import matplotlib.pyplot as plt
import libSense.get_data as lib


class process:
    r"""
    setup to process data
    path, RGB img name and Depth img name + dot + img type,
    intrinsic, voxel_size, method to create rgbd img
       e.g. path =(r"C:\Users\Python\Project")
            RGBimg = "SenseRGB.jpg"
            Depthimg = "SenseDepth.png"
            intrinsic = "intrinsic.json" or intrinsic = [[522.259, 0., 330.18],
                                                         [0., 523.419, 254.437],
                                                         [0., 0., 1.]]
            voxel_size = 0.06
            method to create rgbd img create_from_tum_format
                options: "tum"
                         "color_and_depth"
                         "redwood"
                         "sun"
    """
    def __init__(self, path = None, RGB_name = None, Depth_name = None, point_cloud_name = None,
                 intrinsic = None, voxel_size = None, rgbd = None):
        r"""
        Default values:
            Path = current file directory
            RGB_name = "SenseRGB.jpg"
            Depth_name = "SenseDepth.png"
            point_cloud_name = "point_cloud.pcd"
            voxel_size = 0.0001
            rgbd_image = redwood
        """
        defPathn = os.getcwd()
        defRGBn = "SenseRGB.jpg"
        defDepthn = "SenseDepth.png"
        defPcdn = "point_cloud.pcd"
        defIntrinsic = o3d.camera.PinholeCameraIntrinsic(
                       o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
        defVoxel_size = 0.0001
        defrgbd = "redwood"

        self.path = path if path is not None else defPathn
        self.RGBn = RGB_name if RGB_name is not None else defRGBn
        self.Depthn = Depth_name if Depth_name is not None else defDepthn
        self.pcdn = point_cloud_name if point_cloud_name is not None else defPcdn
        if intrinsic is not None:
            intrinsic = o3d.io.read_pinhole_camera_intrinsic(intrinsic)
        self.intrinsic = intrinsic if intrinsic is not None else defIntrinsic
        self.voxel_size = voxel_size if voxel_size is not None else defVoxel_size
        self.rgbd = rgbd if rgbd is not None else defrgbd

    def get_point_cloud(self, combine = None):
        r"""
            from rgb and depth img creates point cloud
            for use choose: path, rgb_img, depth_img, point_cloud_name.pcd
            and method to create rgbd img
        """
        defcombine = 0
        combine = combine if combine is not None else defcombine
        os.chdir(self.path)
        rgb = o3d.io.read_image(self.RGBn)
        depth = o3d.io.read_image(self.Depthn)

        if self.rgbd == "tum":
            rgbd_image = o3d.geometry.RGBDImage.create_from_tum_format(rgb, depth, convert_rgb_to_intensity=False)
        if self.rgbd == "color_and_depth":
            rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb, depth, convert_rgb_to_intensity=False)
        if self.rgbd == "redwood":
            rgbd_image = o3d.geometry.RGBDImage.create_from_redwood_format(rgb, depth, convert_rgb_to_intensity=False)
        if self.rgbd == "sun":
            rgbd_image = o3d.geometry.RGBDImage.create_from_sun_format(rgb, depth, convert_rgb_to_intensity=False)

        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
              rgbd_image, self.intrinsic)
        # Flip it, otherwise the pointcloud will be upside down
        pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        pcd_down = pcd.voxel_down_sample(voxel_size=self.voxel_size)
        pcd_down.remove_statistical_outlier(nb_neighbors=20,
                                            std_ratio=1.0)
        pcd_down.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.001, max_nn=30))
        if combine == 1:
            o3d.io.write_point_cloud((self.path + "\\" + str(self.pcdn[:-4]) + "2" + str(self.pcdn[-4:])), pcd_down)
        else:
            o3d.io.write_point_cloud((self.path + "\\" + str(self.pcdn[:-4]) + "1" + str(self.pcdn[-4:])), pcd_down)
        return pcd_down

    def load_point_clouds(self):
        r"""
            from path loads two point clouds
        """
        os.chdir(self.path)
        pcds = []
        for i in range(1, 3):
            pcd = o3d.io.read_point_cloud(str(self.pcdn[:-4]) + "%d" % i + str(self.pcdn[-4:]))
            pcds.append(pcd)
        return pcds

    def pairwise_registration(self, source, target, max_correspondence_distance_coarse,
                              max_correspondence_distance_fine):
        icp_coarse = o3d.pipelines.registration.registration_icp(
            source, target, max_correspondence_distance_coarse, np.identity(4),
            o3d.pipelines.registration.TransformationEstimationPointToPlane())
        icp_fine = o3d.pipelines.registration.registration_icp(
            source, target, max_correspondence_distance_fine,
            icp_coarse.transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPlane())
        transformation_icp = icp_fine.transformation
        information_icp = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
            source, target, max_correspondence_distance_fine,
            icp_fine.transformation)
        return transformation_icp, information_icp

    def full_registration(self, pcds, max_correspondence_distance_coarse,
                          max_correspondence_distance_fine):
        pose_graph = o3d.pipelines.registration.PoseGraph()
        odometry = np.identity(4)
        pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))
        n_pcds = len(pcds)
        for source_id in range(n_pcds):
            for target_id in range(source_id + 1, n_pcds):
                transformation_icp, information_icp = self.pairwise_registration(
                    pcds[source_id], pcds[target_id], max_correspondence_distance_coarse,
                    max_correspondence_distance_fine)
                if target_id == source_id + 1:  # odometry case
                    odometry = np.dot(transformation_icp, odometry)
                    pose_graph.nodes.append(
                        o3d.pipelines.registration.PoseGraphNode(
                            np.linalg.inv(odometry)))
                    pose_graph.edges.append(
                        o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                                 target_id,
                                                                 transformation_icp,
                                                                 information_icp,
                                                                 uncertain=False))
                else:  # loop closure case
                    pose_graph.edges.append(
                        o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                                 target_id,
                                                                 transformation_icp,
                                                                 information_icp,
                                                                 uncertain=True))
        return pose_graph

    def combine(self):
        r"""
            loads and combines two point clouds and merge it by calculation

            for use could be choose: path, rgb_img, depth_img, point_cloud_name.pcd,
            intrinsic, voxel_size, method to create rgbd img
        """
        lib.get_data.RGB(self.path, self.RGBn, self.Depthn)
        self.get_point_cloud(combine=1)
        voxel_size=self.voxel_size
        pcds_down = self.load_point_clouds()
        max_correspondence_distance_coarse = voxel_size * 15
        max_correspondence_distance_fine = voxel_size * 1.5
        with o3d.utility.VerbosityContextManager(
                o3d.utility.VerbosityLevel.Debug) as cm:
            pose_graph = self.full_registration(pcds_down,
                                           max_correspondence_distance_coarse,
                                           max_correspondence_distance_fine)

        pcds = self.load_point_clouds()
        pcd_combined = o3d.geometry.PointCloud()
        for point_id in range(len(pcds)):
            pcds[point_id].transform(pose_graph.nodes[point_id].pose)
            pcd_combined += pcds[point_id]
        pcd_combined_down = pcd_combined.voxel_down_sample(voxel_size=voxel_size)
        pcd_combined_down.remove_statistical_outlier(nb_neighbors=20,
                                                     std_ratio=1.0)
        o3d.io.write_point_cloud(str(self.pcdn[:-4]) + "1" + str(self.pcdn[-4:]), pcd_combined_down)
        o3d.io.write_point_cloud(self.pcdn, pcd_combined_down)

        # pcd_combined_down = kd()
        # while True:
        #     if os.path.exists("multi2.pcd") and os.path.getsize("multi2.pcd") > 0:
        #         break
        # o3d.visualization.draw_geometries([pcd_combined_down])
        return pcd_combined_down

    def kd(self):
        r"""
            uses mls "smoothing" filter on point cloud
            for use could be choose: path, point_cloud_name.pcd,
        """
        os.chdir(self.path)
        cloud = pcl.load(str(self.pcdn[:-4]) + "1" + str(self.pcdn[-4:]))

        # // Create a KD-Tree
        tree = cloud.make_kdtree()
        mls = cloud.make_moving_least_squares()
        mls.set_Compute_Normals(True)
        mls.set_polynomial_fit(True)
        mls.set_Search_Method(tree)
        mls.set_search_radius(0.1)#0.03
        mls_points = mls.process()
        # Save output
        pcl.save_PointNormal(mls_points, str(self.pcdn[:-4]) + "1" + str(self.pcdn[-4:]))
        pcd = o3d.io.read_point_cloud(str(self.pcdn[:-4]) + "1" + str(self.pcdn[-4:]))
        return pcd

    def ground_det(self):
        r"""
            detects ground from point cloud
            for use choose: path, point_cloud_name.pcd
        """
        os.chdir(self.path)
        cloud = pcl.load(str(self.pcdn[:-4]) + "1" + str(self.pcdn[-4:]))

        fil = cloud.make_passthrough_filter()
        fil.set_filter_field_name("y")
        fil.set_filter_limits(-1.5, 0)#pasmo
        cloud_filtered = fil.filter()
        print(cloud_filtered.size)
        if cloud_filtered.size > 0:
            seg = cloud_filtered.make_segmenter_normals(ksearch=50)
            seg.set_optimize_coefficients(True)
            seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
            seg.set_normal_distance_weight(0.1)
            seg.set_method_type(pcl.SAC_RANSAC)
            seg.set_max_iterations(100)
            seg.set_distance_threshold(0.03)
            indices, model = seg.segment()

            cloud_plane = cloud_filtered.extract(indices, negative=False)
            pcl.save(cloud_plane, str(self.pcdn[:-4]) + 'plane.pcd')
        floor = o3d.io.read_point_cloud(str(self.pcdn[:-4]) + 'plane.pcd')
        floor.paint_uniform_color([1.0, 0, 0])
        return floor

    def canny(self):
        r"""
            uses on RGB or Depth img canny filter
            for use choose: path, rgb_img, depth_img
        """
        os.chdir(self.path)

        pic1 = plt.imread(self.RGBn)
        pic1 = np.uint8(pic1)
        pic2 = o3d.io.read_image(self.Depthn)
        pic2 = np.uint8(pic2)

        hist = np.histogram(pic1, bins=np.arange(0, 256))
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(8, 3))
        ax1.imshow(pic1, cmap=plt.cm.gray, interpolation='nearest')
        ax1.axis('off')
        ax2.plot(hist[1][:-1], hist[0], lw=2)
        ax2.set_title('histogram of grey values')

        plt.show()

        img = pic1
        edges = cv2.Canny(img, 100, 200)

        plt.subplot(121),plt.imshow(img,cmap = 'gray')
        plt.title('Original Image'), plt.xticks([]), plt.yticks([])
        plt.subplot(122),plt.imshow(edges,cmap = 'gray')
        plt.title('Edge Image'), plt.xticks([]), plt.yticks([])

        plt.show()

    def median_filter(self):
        os.chdir(self.path)
        img = cv2.imread(self.Depthn)
        # depth_median = cv2.medianBlur(img, 5)
        depth_median = cv2.bilateralFilter(img,1,15,15)#(img,9,75,75)
        img_save = cv2.imwrite(str(self.Depthn[:-4]) + "_F" + str(self.Depthn[-4:]), depth_median)