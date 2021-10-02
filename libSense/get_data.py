import open3d as o3d
import pyrealsense2 as rs
from numpy import *
import numpy as np
import cv2
import pcl
import os

class get_data:
    r"""
    Note
    ----
    Setup to save data
    Callable functions:
        RGB() to get RGB and Depth and save it.
        RGBLive() to get live stream of RGB and depth with save option.

    Parameters
    ----------
    Path : str
        directory path
    RGB_name : str
        RGB image name
    Depth_name : str
        Depth image name

    Examples
    --------
    path = (r"C:\Users\Python\Project")
    RGBimg = "SenseRGB.jpg"
    Depthimg = "SenseDepth.png"
    """
    def __init__(self, path = None, RGB_name = None, Depth_name = None):
        r"""
        Note
        ----
        Default values:
            Path = current file directory
            RGB_name = "SenseRGB.jpg"
            Depth_name = "SenseDepth.png"
        """
        defPathn = os.getcwd()
        defRGBn = "SenseRGB.jpg"
        defDepthn = "SenseDepth.png"

        self.path = path if path is not None else defPathn
        self.RGBn = RGB_name if RGB_name is not None else defRGBn
        self.Depthn = Depth_name if Depth_name is not None else defDepthn

    def startRGB(self):
        r"""
        Note
        ----
        config RealSense to stream
        """
        # Configure depth and color streams
        pipeline = rs.pipeline()
        config = rs.config()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        found_rgb = False
        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            print("Requires Depth camera with Color sensor")
            exit(0)

        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        pipeline.start(config)
        return pipeline

    def getColorImage(self, frame):
        """
        Note
        ----
        gets img and saves it
        """
        global r
        path = (self.path + "\\" + self.RGBn)
        if r == False:
            rgb = np.asanyarray(frame.get_data())
            cv2.imwrite(path, rgb)
            r = True

    def getDepthImage(self, frame):
        """
        Note
        ----
        gets img and saves it
        """
        global d
        path = (self.path + "\\" + self.Depthn)
        if d == False:
            depth = np.asanyarray(frame.get_data())
            cv2.imwrite(path, depth)
            d = True

    def RGB(self):
        """
        Note
        ----
        Callable function
        Gets img RGB and Depth and saves it.
        """
        pipeline = self.startRGB()
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        # RGB
        global r
        r = False
        color_frame = frames.get_color_frame()
        self.getColorImage(color_frame)
        # RGB-D
        global d
        d = False
        depth_frame = frames.get_depth_frame()
        self.getDepthImage(depth_frame)
        while True:
            if d == True:
                break

    def RGBLive(self):
        """
        Note
        ----
        Callable function
        Gets live stream of RGB and Depth.
        For save data press 's'.
        """
        pipeline = self.startRGB()
        print("Press 's' to save images.")
        try:
            while True:

                # Wait for a coherent pair of frames: depth and color
                frames = pipeline.wait_for_frames()
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                if not depth_frame or not color_frame:
                    continue

                # Convert images to numpy arrays
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

                depth_colormap_dim = depth_colormap.shape
                color_colormap_dim = color_image.shape

                # If depth and color resolutions are different, resize color image to match depth image for display
                if depth_colormap_dim != color_colormap_dim:
                    resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]),
                                                     interpolation=cv2.INTER_AREA)
                    images = np.hstack((resized_color_image, depth_colormap))
                else:
                    images = np.hstack((color_image, depth_colormap))

                # Show images
                cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                cv2.imshow('RealSense', images)
                cv2.waitKey(1)

                if cv2.waitKey(115) == ord('s'):
                    # self.RGB()
                    print("pressed s")


        finally:

            # Stop streaming
            pipeline.stop()