#!/usr/bin/env python

from pcl_helper import *


def pcl_callback(pcl_msg):

    # TODO: Initialization
    cloud = pcl.PointCloud_PointXYZRGB()

    # TODO: Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)

    # print cloud size
    # print cloud.size

    # TODO: Voxel Grid
    vox = cloud.make_voxel_grid_filter()
    vox.set_leaf_size(0.005, 0.005, 0.005)
    cloud_filtered = vox.filter()

    # TODO: Filter out points close to the ground
    passthrough = cloud_filtered.make_passthrough_filter()
    passthrough.set_filter_field_name("z")
    passthrough.set_filter_limits(0.77, 2.0)
    cloud_filtered = passthrough.filter()

    # TODO: RANSAC Plane Segmentation
    seg = cloud_filtered.make_segmenter()
    seg.set_optimize_coefficients(True)
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_distance_threshold(.01)
    inliers, coefficients = seg.segment()

    # Extract inliers
    cloud_table = cloud_filtered.extract(inliers, negative=False)

    # Extract outliers
    cloud_out = cloud_filtered.extract(inliers, negative=True)

    # pcl.save(cloud_table, 'in.pcd')

    # Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_out)
    tree = white_cloud.make_kdtree()
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.01)
    ec.set_MinClusterSize(100)
    ec.set_MaxClusterSize(25000)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()

    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_color = get_color_list(len(cluster_indices))

    print 'cluster color len: ', len(cluster_color)

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        print 'indices = ', j, len(indices)
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                            white_cloud[indice][1],
                                            white_cloud[indice][2],
                                            rgb_to_float(cluster_color[j])])

    cluster_cloud.from_list(color_cluster_point_list)

    # TODO: Convert PCL data to ROS msg
    ros_cloud_out = pcl_to_ros(cloud_out)
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cluster = pcl_to_ros(cluster_cloud)

    # TODO: Publish ROS msg
    pcl_out_pub.publish(ros_cloud_out)
    pcl_in_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster)


if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud", pc2.PointCloud2, pcl_callback, queue_size=1)

    # TODO: Create Publishers
    pcl_in_pub = rospy.Publisher("/pcl_in", PointCloud2, queue_size=1)
    pcl_out_pub = rospy.Publisher("/pcl_out", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)

    # Initialize color_list
    get_color_list.color_list = []
    while not rospy.is_shutdown():
        rospy.spin()