#! /usr/bin/env python3
import sys
import numpy as np
from cv2 import cv2
import open3d as o3d
from pyrealsense2 import pyrealsense2 as rs


class GetRealSenseData():

  def __init__(self):
    self.__pipeline = rs.pipeline()
    self.__config = rs.config()
    self.__config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    self.__config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    # process
    self.__align_depth2color = rs.align(rs.stream.color)
    self.__pc = rs.pointcloud()   # get pointcloud
    # frame data
    self.depth_image = None
    self.depth_colormap = None
    self.color_image = None
    self.verts = None           # xyz
    self.texcoords = None       # u,v
    # start streaming
    self.__pipeline.start(self.__config)
    # camera intrinsic
    self.__profile = self.__pipeline.get_active_profile()
    self.intr = self.__profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

  # ====================== interface ======================
  def stream_color_frame(self):
    frames = self.__pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    if color_frame:
      # Convert images to numpy arrays
      self.color_image = np.asanyarray(color_frame.get_data())
    return color_frame

  def stream_depth_frame(self):
    frames = self.__pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    # apply depth filters
    depth_frame = self.depth_filter(depth_frame)
    if depth_frame:
      # Convert images to numpy arrays
      self.depth_image = np.asanyarray(depth_frame.get_data())
      self.depth_colormap = cv2.applyColorMap(
          cv2.convertScaleAbs(self.depth_image, alpha=0.03), cv2.COLORMAP_JET)
    return depth_frame

  def stream_depth2color_aligned(self):
    frames = self.__pipeline.wait_for_frames()
    # align depth to color frame
    aligned_frames = self.__align_depth2color.process(frames)
    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    # apply depth filters
    depth_frame = self.depth_filter(depth_frame)
    if depth_frame and color_frame:
      self.depth_image = np.asanyarray(depth_frame.get_data())
      self.depth_colormap = cv2.applyColorMap(
          cv2.convertScaleAbs(self.depth_image, alpha=0.03), cv2.COLORMAP_JET)
      self.color_image = np.asanyarray(color_frame.get_data())
    return depth_frame, color_frame

  def stream_pointcloud(self):
    depth_frame, color_frame = self.stream_depth2color_aligned()
    points = self.__pc.calculate(depth_frame)
    self.__pc.map_to(depth_frame)
    # Pointcloud data to arrays
    v, t = points.get_vertices(), points.get_texture_coordinates()
    self.verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)   # xyz
    self.texcoords = np.asanyarray(t).view(np.float32).reshape(-1, 2)  # uv
    return depth_frame, color_frame

  def stop_stream(self):
    self.__pipeline.stop()

  # ====================== utility ======================
  def depth_filter(self, depth_frame):
    depth_frame = rs.decimation_filter(1).process(depth_frame)
    depth_frame = rs.disparity_transform(True).process(depth_frame)
    depth_frame = rs.spatial_filter().process(depth_frame)
    depth_frame = rs.temporal_filter().process(depth_frame)
    depth_frame = rs.disparity_transform(False).process(depth_frame)
    return depth_frame

  def getPoint(depth_frame, pixels):
    depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
    points = []
    for i in range(len(pixels)):
      pix = pixels[i]
      try:
        depth_in_met = depth_frame.as_depth_frame().get_distance(pix[1], pix[0])
        pnt = rs.rs2_deproject_pixel_to_point(depth_intrin, pix, depth_in_met)
      except Exception as err:
        print(err)
        pnt = [0.0, 0.0, 0.0]
      points.append(pnt)
    points_formatted = np.reshape(points, [3, len(pixels)]).T
    # points_formatted = np.array(points).flatten()
    return points_formatted


# exit
def breakLoop(vis):
  vis.destroy_window()
  cv2.destroyAllWindows()
  sys.exit()


# test case: visualize pointcloud using open3d
def test():
  get_realsense_data = GetRealSenseData()

  pointcloud = o3d.geometry.PointCloud()
  cam_intr = o3d.camera.PinholeCameraIntrinsic(
      get_realsense_data.intr.width, get_realsense_data.intr.height,
      get_realsense_data.intr.fx, get_realsense_data.intr.fy,
      get_realsense_data.intr.ppx, get_realsense_data.intr.ppy)
  vis = o3d.visualization.VisualizerWithKeyCallback()
  vis.create_window("pointcloud", 640, 480)
  vis.register_key_callback(ord("Q"), breakLoop)
  isGeometryAdded = False

  cv2.namedWindow('RGB-depth', cv2.WINDOW_AUTOSIZE)
  while True:
    get_realsense_data.stream_depth2color_aligned()
    color_depth_stack = np.vstack((get_realsense_data.color_image, get_realsense_data.depth_colormap))
    cv2.imshow('RGB-depth', color_depth_stack)
    key = cv2.waitKey(10)
    if key & 0xFF == ord('Q') or key == 27:
      get_realsense_data.stop_stream()
      breakLoop(vis)
      break

    pointcloud.clear()
    depth = o3d.geometry.Image(get_realsense_data.depth_image)
    rgb = cv2.cvtColor(get_realsense_data.color_image, cv2.COLOR_RGB2BGR)
    color = o3d.geometry.Image(rgb)

    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color, depth, convert_rgb_to_intensity=False)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, cam_intr)
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    pointcloud += pcd
    if not isGeometryAdded:
      vis.add_geometry(pointcloud)
      isGeometryAdded = True
    # update geometry
    vis.update_geometry(pointcloud)
    vis.poll_events()
    vis.update_renderer()


if __name__ == "__main__":
  test()
