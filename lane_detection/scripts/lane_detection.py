#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy

import numpy as np
import ros_numpy
import colorsys
import struct
import message_filters
import ctypes

import open3d as o3d
import tf as tf2
from tf import TransformListener

from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from tf2_msgs.msg import TFMessage

type_mappings = [(PointField.INT8, np.dtype('int8')), (PointField.UINT8, np.dtype('uint8')), (PointField.INT16, np.dtype('int16')),
                 (PointField.UINT16, np.dtype('uint16')), (PointField.INT32, np.dtype('int32')), (PointField.UINT32, np.dtype('uint32')),
                 (PointField.FLOAT32, np.dtype('float32')), (PointField.FLOAT64, np.dtype('float64'))]
pftype_to_nptype = dict(type_mappings)

pftype_sizes = {PointField.INT8: 1, PointField.UINT8: 1, PointField.INT16: 2, PointField.UINT16: 2,
                 PointField.INT32: 4, PointField.UINT32: 4, PointField.FLOAT32: 4, PointField.FLOAT64: 8}

_DATATYPES = {}
_DATATYPES[PointField.INT8]    = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)

DUMMY_FIELD_PREFIX = '__'

# calib = None
# P_velo2cam_ref = None
# pc_stack = np.empty((0, 4), float)
pc_stack = np.empty((0, 3), float)

odom = []

frame_stack = 5

class lane_detection_class:

    def __init__(self):
        self.lidar_pub = rospy.Publisher("/frame_stack", PointCloud2, queue_size=4)
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=4)

        self.odom_sub = message_filters.Subscriber("/aft_mapped_to_init", Odometry, queue_size=4)
        self.lidar_sub = message_filters.Subscriber("/ground_cloud_intensity2", PointCloud2, queue_size=4)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.odom_sub, self.lidar_sub], 3, 0.1, allow_headerless=True)
        self.ts.registerCallback(self.calib_callback)
        
        self.tf = TransformListener()

    def calib_callback(self, Odometry, PointCloud2):
        global pc_stack, frame_stack

        pc_np = get_xyzi_points(pointcloud2_to_array(PointCloud2))

        
        # if PointCloud2.header.seq == 0:
        #     pc_stack = np.empty((0, 4), float)
        #     print("init")

        # pc_np = get_xyz_points(pointcloud2_to_array(PointCloud2))

        # if PointCloud2.header.seq == 0:
        #     pc_stack = np.empty((0, 3), float)
        #     print("init")

        # print("callback : ", PointCloud2.header.seq)

        # pc_stack = extract_points(pc_stack, voxel_size = 0.001, x_range= (-1000, 1000), y_range= (-1000, 1000), z_range= (-10, 10), i_range= (0, 1))
        # pc_stack = extract_points(pc_stack)

        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pc_np[:,:3])

        pts_3d = np.asarray(pcd.points).astype(np.float32)

        odom_mat = self.get_odom()

        if odom_mat is not None:
            odom_mat = odom_mat[:3, :]
            num_pts = pts_3d.transpose().shape[1]
            pts_3d = np.vstack((pts_3d.transpose(), np.ones((1, num_pts))))
            pts_3d = np.matmul(odom_mat, pts_3d)
            pts_3d = pts_3d.transpose()
            pc_stack = np.append(pc_stack, pts_3d, axis=0)
            PointCloud2.header.frame_id = "/map"
            point_pc2 = create_cloud_xyz(PointCloud2.header, pc_stack)
            self.lidar_pub.publish(point_pc2)

        # pc_stack = np.append(pc_stack, pc_np, axis=0)
        # print(odom_mat.size)
        #     points = np.vstack((points, np.ones((1, num_pts))))
        #     points = np.matmul(proj_mat, points)

        # if (PointCloud2.header.seq > 0) and (PointCloud2.header.seq % frame_stack == 0): 

        #     point_pc2 = create_cloud_xyzi(PointCloud2.header, pc_stack)
        #     pc_stack = np.empty((0, 4), float)

        #     print("pub : ", point_pc2.header.seq)
        #     self.lidar_pub.publish(point_pc2)

    def get_odom(self):
        try:
            t = self.tf.getLatestCommonTime("/map", "/velo_link")
            position, quaternion = self.tf.lookupTransform("/map", "/velo_link", t)

            trans_mat = tf2.transformations.translation_matrix(position)
            rot_mat = tf2.transformations.quaternion_matrix(quaternion)
            # create a 4x4 matrix
            mat = np.dot(trans_mat, rot_mat)
            # rotation = tf2.transformations.euler_from_quaternion(quaternion)
        except(tf2.Exception, tf2.ConnectivityException, tf2.LookupException):
            return
        return mat


def _get_struct_fmt(is_bigendian, fields, field_names=None):
    fmt = '>' if is_bigendian else '<'

    offset = 0
    for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print('Skipping unknown PointField datatype [%d]' % field.datatype, file=sys.stderr)
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt    += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt

def create_cloud_xyzi(header, points):
    """
    Create a L{sensor_msgs.msg.PointCloud2} message with 3 float32 fields (x, y, z).

    @param header: The point cloud header.
    @type  header: L{std_msgs.msg.Header}
    @param points: The point cloud points.
    @type  points: iterable
    @return: The point cloud.
    @rtype:  L{sensor_msgs.msg.PointCloud2}
    """
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              PointField('intensity', 12, PointField.FLOAT32, 1)]
    return create_cloud(header, fields, points)

def create_cloud_xyz(header, points):
    """
    Create a L{sensor_msgs.msg.PointCloud2} message with 3 float32 fields (x, y, z).

    @param header: The point cloud header.
    @type  header: L{std_msgs.msg.Header}
    @param points: The point cloud points.
    @type  points: iterable
    @return: The point cloud.
    @rtype:  L{sensor_msgs.msg.PointCloud2}
    """
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1)]
    return create_cloud(header, fields, points)

def create_cloud(header, fields, points):
    """
    Create a L{sensor_msgs.msg.PointCloud2} message.

    @param header: The point cloud header.
    @type  header: L{std_msgs.msg.Header}
    @param fields: The point cloud fields.
    @type  fields: iterable of L{sensor_msgs.msg.PointField}
    @param points: The point cloud points.
    @type  points: list of iterables, i.e. one iterable for each point, with the
                   elements of each iterable being the values of the fields for 
                   that point (in the same order as the fields parameter)
    @return: The point cloud.
    @rtype:  L{sensor_msgs.msg.PointCloud2}
    """

    cloud_struct = struct.Struct(_get_struct_fmt(False, fields))

    buff = ctypes.create_string_buffer(cloud_struct.size * len(points))

    point_step, pack_into = cloud_struct.size, cloud_struct.pack_into
    offset = 0
    for p in points:
        pack_into(buff, offset, *p)
        offset += point_step

    return pc2.PointCloud2(header=header,
                       height=1,
                       width=len(points),
                       is_dense=False,
                       is_bigendian=False,
                       fields=fields,
                       point_step=cloud_struct.size,
                       row_step=cloud_struct.size * len(points),
                       data=buff.raw)

def fields_to_dtype(fields, point_step):
    '''Convert a list of PointFields to a numpy record datatype.
    '''
    offset = 0
    np_dtype_list = []
    for f in fields:
        while offset < f.offset:
            # might be extra padding between fields
            np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
            offset += 1

        dtype = pftype_to_nptype[f.datatype]
        if f.count != 1:
            dtype = np.dtype((dtype, f.count))

        np_dtype_list.append((f.name, dtype))
        offset += pftype_sizes[f.datatype] * f.count

    # might be extra padding between points
    while offset < point_step:
        np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
        offset += 1
        
    return np_dtype_list

def pointcloud2_to_array(cloud_msg, squeeze=True):
    ''' Converts a rospy PointCloud2 message to a numpy recordarray 
    
    Reshapes the returned array to have shape (height, width), even if the height is 1.

    The reason for using np.fromstring rather than struct.unpack is speed... especially
    for large point clouds, this will be <much> faster.
    '''
    # construct a numpy record type equivalent to the point type of this cloud
    dtype_list = fields_to_dtype(cloud_msg.fields, cloud_msg.point_step)

    # parse the cloud into an array
    cloud_arr = np.frombuffer(cloud_msg.data, dtype_list)

    # remove the dummy fields that were added
    cloud_arr = cloud_arr[
        [fname for fname, _type in dtype_list if not (fname[:len(DUMMY_FIELD_PREFIX)] == DUMMY_FIELD_PREFIX)]]
    
    if squeeze and cloud_msg.height == 1:
        return np.reshape(cloud_arr, (cloud_msg.width,))
    else:
        return np.reshape(cloud_arr, (cloud_msg.height, cloud_msg.width))

def get_xyzi_points(cloud_array, remove_nans=True, dtype=np.float):
    '''Pulls out x, y, and z columns from the cloud recordarray, and returns
        a 3xN matrix.
    '''
    # remove crap points
    # if remove_nans:
    #     mask_ = np.isfinite(cloud_array['x']) & np.isfinite(cloud_array['y']) & np.isfinite(cloud_array['z'] & np.isfinite(cloud_array['intensity'])
    #     cloud_array = cloud_array[mask_]
    
    # pull out x, y, and z values
    points = np.zeros(cloud_array.shape + (4,), dtype=dtype)
    points[...,0] = cloud_array['x']
    points[...,1] = cloud_array['y']
    points[...,2] = cloud_array['z']
    points[...,3] = cloud_array['intensity']

    return points

def get_xyz_points(cloud_array, remove_nans=True, dtype=np.float):
    '''Pulls out x, y, and z columns from the cloud recordarray, and returns
        a 3xN matrix.
    '''
    # remove crap points
    # if remove_nans:
    #     mask_ = np.isfinite(cloud_array['x']) & np.isfinite(cloud_array['y']) & np.isfinite(cloud_array['z'] & np.isfinite(cloud_array['intensity'])
    #     cloud_array = cloud_array[mask_]
    
    # pull out x, y, and z values
    points = np.zeros(cloud_array.shape + (4,), dtype=dtype)
    points[...,0] = cloud_array['x']
    points[...,1] = cloud_array['y']
    points[...,2] = cloud_array['z']

    return points

def extract_points(points, voxel_size = 0.01, x_range= (-30, 30), y_range= (-30, 30), z_range= (-10, 10), i_range= (0.5, 0.9)):

    x, y, z, i = points[:, 0], points[:, 1], points[:, 2], points[:, 3]

    
    in_range = box_in_range(x,y,z,i, x_range, y_range, z_range, i_range)

    points = points[in_range]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:,:3])
    
    pcd = pcd.voxel_down_sample(voxel_size)

    return pcd

def box_in_range(x,y,z,i, x_range, y_range, z_range, i_range):
    """ extract filtered in-range velodyne coordinates based on x,y,z limit """
    return np.logical_and.reduce((
            x > x_range[0], x < x_range[1],
            y > y_range[0], y < y_range[1],
            z > z_range[0], z < z_range[1],
            i > i_range[0], i < i_range[1]))


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
# def cal_proj_mat_PRT(calib):
#     P_velo2cam_ref = np.vstack((calib['Tr_velo_to_cam'].reshape(3, 4), np.array([0., 0., 0., 1.])))  # velo2ref_cam
#     R_ref2rect = np.eye(4)
#     R0_rect = calib['R0_rect'].reshape(3, 3)  # ref_cam2rect
#     R_ref2rect[:3, :3] = R0_rect
#     P_rect2cam2 = calib['P2'].reshape((3, 4))
#     proj_mat = np.matmul(np.matmul(P_rect2cam2, R_ref2rect), P_velo2cam_ref)
#     return proj_mat

# def project_to_image(points, proj_mat):
#     """
#     Apply the perspective projection
#     Args:
#         pts_3d:     3D points in camera coordinate [3, npoints]
#         proj_mat:   Projection matrix [3, 4]

#     """
#     num_pts = points.shape[1]

#     # Change to homogenous coordinate
#     points = np.vstack((points, np.ones((1, num_pts))))
#     points = np.matmul(proj_mat, points)
#     points[:2, :] /= points[2, :]
#     return points[:2, :]

def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('Lane_detection_node', anonymous=True)

    # global calib, P_velo2cam_ref
    # # Load calibration
    # #calib = read_calib_file('/home/yg-ubuntu1804/calib2.txt')
    # calib = read_calib_file('/home/user/catkin_ws/src/lane_detection/calib_data/calib.txt')
    # P_velo2cam_ref = np.vstack((calib['Tr_velo_to_cam'].reshape(3, 4), np.array([0., 0., 0., 1.])))  # velo2ref_cam

    # # projection matrix (project from velo2cam2)
    # proj_velo2cam2 = cal_proj_mat_PRT(calib)

    lane_detection = lane_detection_class()

    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
