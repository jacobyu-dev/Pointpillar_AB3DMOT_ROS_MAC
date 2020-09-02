#!/usr/bin/env python
import sys
import rospy

import cv2
import numpy as np
import ros_numpy
import colorsys

import message_filters

import open3d as o3d

from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2

from cv_bridge import CvBridge, CvBridgeError

calib = None
proj_velo2cam2 = None

class calib_feature:

    def __init__(self):
        self.bridge = CvBridge()
        self.cv_image = None
        self.image_pub = rospy.Publisher("calib_img", Image, queue_size=1)
        self.image_sub = message_filters.Subscriber("/kitti/camera_color_left/image_raw", Image)
        self.lidar_sub = message_filters.Subscriber("/kitti/velo/pointcloud", PointCloud2)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.lidar_sub], 3, 0.1, allow_headerless=True)
        self.ts.registerCallback(self.calib_callback)

    def calib_callback(self, Image, PointCloud2):

        ori_img = self.bridge.imgmsg_to_cv2(Image, "bgr8")

        # img_height, img_width, img_channel = ori_img.shape         
        
        pc_velo =  ros_numpy.point_cloud2.pointcloud2_to_xyz_array(PointCloud2)

        #pcd = extract_points(pc_velo,0.1)
        
        pcd = extract_points(pc_velo,0.01)
        pts_3d = np.asarray(pcd.points).astype(np.float32)

        # Filter out the points that are behind us.
        pts_3d= get_in_view_pts(pcd)

        ori_img = project_3d_to_2d(ori_img, pts_3d)
        
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(ori_img, "bgr8"))
        #self.lidar_pub.publish(ros_data)

def project_3d_to_2d(img, pts_3d):
    global proj_velo2cam2

    image = img.copy()

    # # apply projection
    pts_2d = project_to_image(pts_3d.transpose(), proj_velo2cam2)

    x, y, z = pts_3d[:, 0], pts_3d[:, 1], pts_3d[:, 2]
    dist = np.sqrt(x ** 2 + y ** 2 + z ** 2)
    dist_normalize = (dist - dist.min()) / (dist.max() - dist.min())
    color = [[int(x*255) for x in colorsys.hsv_to_rgb(hue,1,1)] for hue in dist_normalize]

    tmp_pts_2d_x = pts_2d[0, :].astype(np.int32).tolist()
    tmp_pts_2d_y = pts_2d[1, :].astype(np.int32).tolist()

    for (x,y,c) in zip(tmp_pts_2d_x,tmp_pts_2d_y,color):
        cv2.circle(image, (x, y), 2, [c[2],c[1],c[0]], -1)

    return image

def get_in_view_pts(pcd):
    """ 
        Convert open3d.geometry.PointCloud object to [4, N] array
                    [x_1 , x_2 , .. ]
        xyz_v   =   [y_1 , y_2 , .. ]
                    [z_1 , z_2 , .. ]
                    [ 1  ,  1  , .. ]
    """
    # The [N,3] downsampled array
    pts_3d = np.asarray(pcd.points)

    # finter out the points not in view
    h_points = hv_in_range(pts_3d[:,0], pts_3d[:,1], [-50,50], fov_type='h')
    pts_3d = pts_3d[h_points]

    return pts_3d

def extract_points(points,voxel_size = 0.01, x_range= (-10000, 10000), y_range= (-10000, 10000), z_range= (-10000, 10000),d_range= (-10000, 10000)):
    # filter in range points based on fov, x,y,z range setting
    combined = points_basic_filter(points)
    points = points[combined]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:,:3])

    # approximate_class must be set to true
    # see this issue for more info https://github.com/intel-isl/Open3D/issues/1085
    min_bound = [x_range[0], y_range[0], z_range[0]]
    max_bound = [x_range[1], y_range[1], z_range[1]]
    
    pcd, trace = pcd.voxel_down_sample_and_trace(voxel_size, min_bound, max_bound, approximate_class=True)

    return pcd

def points_basic_filter(points, h_fov=(-180, 180), v_fov=(-25, 2), x_range= (-10000, 10000), y_range= (-10000, 10000), z_range= (-10000, 10000),d_range= (-10000, 10000)):
    """
        filter points based on h,v FOV and x,y,z distance range.
        x,y,z direction is based on velodyne coordinates
        1. azimuth & elevation angle limit check
        2. x,y,z distance limit
        return a bool array
    """
    #assert points.shape[1] == 4, points.shape # [N,3]
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    d = np.sqrt(x ** 2 + y ** 2 + z ** 2) # this is much faster than d = np.sqrt(np.power(points,2).sum(1))

    # # extract in-range fov points
    h_points = hv_in_range(x, y, h_fov, fov_type='h')
    v_points = hv_in_range(d, z, v_fov, fov_type='v')
    combined = np.logical_and(h_points, v_points)

    # # extract in-range x,y,z points
    in_range = box_in_range(x,y,z,d, x_range, y_range, z_range, d_range)
    combined = np.logical_and(combined, in_range)

    return combined

def box_in_range(x,y,z,d, x_range, y_range, z_range, d_range):
    """ extract filtered in-range velodyne coordinates based on x,y,z limit """
    return np.logical_and.reduce((
            x > x_range[0], x < x_range[1],
            y > y_range[0], y < y_range[1],
            z > z_range[0], z < z_range[1],
            d > d_range[0], d < d_range[1]))

def hv_in_range(m, n, fov, fov_type='h'):
    """ extract filtered in-range velodyne coordinates based on azimuth & elevation angle limit 
        horizontal limit = azimuth angle limit
        vertical limit = elevation angle limit
    """
    if fov_type == 'h':
        return np.logical_and(np.arctan2(n, m) > (-fov[1] * np.pi / 180), \
                                np.arctan2(n, m) < (-fov[0] * np.pi / 180))
    elif fov_type == 'v':
        return np.logical_and(np.arctan2(n, m) < (fov[1] * np.pi / 180), \
                                np.arctan2(n, m) > (fov[0] * np.pi / 180))
    else:
        raise NameError("fov type must be set between 'h' and 'v' ")

def read_calib_file(filepath):
    data = {}
    with open(filepath, 'r') as f:
        for line in f.readlines():
            line = line.rstrip()
            if len(line) == 0: continue
            key, value = line.split(':', 1)
            # The only non-float values in these files are dates, which
            # we don't care about anyway
            try:
                data[key] = np.array([float(x) for x in value.split()])
            except ValueError:
                pass

    return data


# =========================================================
# Projections
# =========================================================
def cal_proj_mat_PRT(calib):
    P_velo2cam_ref = np.vstack((calib['Tr_velo_to_cam'].reshape(3, 4), np.array([0., 0., 0., 1.])))  # velo2ref_cam
    R_ref2rect = np.eye(4)
    R0_rect = calib['R0_rect'].reshape(3, 3)  # ref_cam2rect
    R_ref2rect[:3, :3] = R0_rect
    P_rect2cam2 = calib['P2'].reshape((3, 4))
    proj_mat = np.matmul(np.matmul(P_rect2cam2, R_ref2rect), P_velo2cam_ref)
    return proj_mat

def project_to_image(points, proj_mat):
    """
    Apply the perspective projection
    Args:
        pts_3d:     3D points in camera coordinate [3, npoints]
        proj_mat:   Projection matrix [3, 4]

    """
    num_pts = points.shape[1]

    # Change to homogenous coordinate
    points = np.vstack((points, np.ones((1, num_pts))))
    points = np.matmul(proj_mat, points)
    points[:2, :] /= points[2, :]
    return points[:2, :]

def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('Lidar2cam_calib_node', anonymous=True)

    global calib, proj_velo2cam2
    # Load calibration
    #calib = read_calib_file('/home/yg-ubuntu1804/calib2.txt')
    calib = read_calib_file('calib_data/calib.txt')


    # # projection matrix (project from velo2cam2)
    proj_velo2cam2 = cal_proj_mat_PRT(calib)

    calib_class = calib_feature()

    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
