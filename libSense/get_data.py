import usb.util
import open3d as o3d
import pyrealsense2 as rs
from numpy import *
import numpy as np
import cv2
import pcl
import win32com.client

class get_data:
    r"""
    setup to save data
    path and names of RGB img and Depth img + dot + img type
       e.g. path =(r"C:\Users\Python\Project")
            RGBimg = "SenseRGB.jpg"
            Depthimg = "SenseDepth.png"
    """
    def __init__(self, path = None, RGB_name = None, Depth_name = None):
        r"""
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

#########dunno if this part works#####################
    def startRGB():
        r"""config RealSense to stream"""
        dev = usb.core.find(idVendor=0x045e, idProduct=0x02B0)

        pipeline = rs.pipeline()
        config = rs.config()

        intrinsic = o3d.camera.PinholeCameraIntrinsic(
                    o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)#toto asi nepotrebuji

        wmi = win32com.client.GetObject("winmgmts:")# je to potreba?

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        pipeline.start(config)
        return pipeline
##################################################################
    def getColorImage(self, frame):
        """gets img and saves it"""
        global r
        path = (self.path + "\\" + self.RGBn)
        if r == False:
            rgb = np.asanyarray(frame.get_data())
            cv2.imwrite(path, rgb)
            r = True

    def getDepthImage(self, frame):
        """gets img and saves it"""
        global d
        path = (self.path + "\\" + self.Depthn)
        if d == False:
            depth = np.asanyarray(frame.get_data())
            cv2.imwrite(path, depth)
            d = True

    def RGB(self):
        """gets imgs RGB and Depth"""
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
