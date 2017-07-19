#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt

from pcl_helper import *

def plot_histogram(rhist, ghist, bhist):
    # Generating bin centers
    bin_edges = rhist[1]
    bin_centers = (bin_edges[1:]  + bin_edges[0:len(bin_edges)-1])/2

    # Plot a figure with all three bar charts
    fig = plt.figure(figsize=(12,3))
    plt.subplot(131)
    plt.bar(bin_centers, rhist[0])
    plt.xlim(0, 256)
    plt.title('R Histogram')
    plt.subplot(132)
    plt.bar(bin_centers, ghist[0])
    plt.xlim(0, 256)
    plt.title('G Histogram')
    plt.subplot(133)
    plt.bar(bin_centers, bhist[0])
    plt.xlim(0, 256)
    plt.title('B Histogram')
    plt.show()

def compute_histograms_for_object_cluster(object_cluster):
    # Compute histograms for the clusters
    object_cluster_arr = object_cluster.to_array()
    packed_colors = object_cluster_arr[:,3]

    r_vals = []
    g_vals = []
    b_vals = []

    for packed_color in np.nditer(packed_colors):
         (r_val, g_val, b_val) = float_to_rgb(packed_color)
         r_vals.append(r_val)
         g_vals.append(g_val)
         b_vals.append(b_val)

    rhist = np.histogram(r_vals, bins=32, range=(0, 256))
    ghist = np.histogram(g_vals, bins=32, range=(0, 256))
    bhist = np.histogram(b_vals, bins=32, range=(0, 256))

    plot_histogram(rhist, ghist, bhist)


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

    #Compute histogram for each object cluster
    global count
    if count:
        return
    count = 1

    for cluster in cluster_indices:
        cluster_with_color = cloud_out.extract(cluster)
        compute_histograms_for_object_cluster(cluster_with_color)
        return

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
    count = 0

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