#!/usr/bin/env python
import rospy
import rospkg
import tf
import random
import math
import numpy as np
import pickle
import pprint
from pcl_helper import *
from sensor_stick.srv import GetNormals
from gazebo_msgs.srv import GetPhysicsProperties
from gazebo_msgs.srv import SetPhysicsProperties
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import SetModelStateRequest
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import GetWorldProperties
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2
import matplotlib.pyplot as plt

def intial_setup():
    # Turn off gravity in Gazebo
    get_physics_properties_prox = rospy.ServiceProxy('gazebo/get_physics_properties', GetPhysicsProperties)
    physics_properties = get_physics_properties_prox()

    physics_properties.gravity.z = 0

    set_physics_properties_prox = rospy.ServiceProxy('gazebo/set_physics_properties', SetPhysicsProperties)
    set_physics_properties_prox(physics_properties.time_step,
                                physics_properties.max_update_rate,
                                physics_properties.gravity,
                                physics_properties.ode_config)

    # That pesky ground plane will only produce extra points in the cloud
    delete_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
    delete_model_prox('ground_plane')


def spawn_model(model_name):
    initial_pose = Pose()
    initial_pose.position.x = 0
    initial_pose.position.y = 1
    initial_pose.position.z = 1

    # Delete the old model if it's stil around
    delete_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
    delete_model_prox('training_model')

    # Also delete the ground plane, it will produce extra erroneous points
    delete_model_prox('ground_plane')

    # Spawn the new model #
    model_path = rospkg.RosPack().get_path('sensor_stick')+'/models/'
    model_xml = ''

    with open (model_path + model_name + '/model.sdf', 'r') as xml_file:
        model_xml = xml_file.read().replace('\n', '')

    spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_prox('training_model', model_xml, '', initial_pose, 'world')

def capture_sample():
    ### Capture Data ###
    get_model_state_prox = rospy.ServiceProxy('gazebo/get_model_state',GetModelState)
    model_state = get_model_state_prox('training_model','world')

    set_model_state_prox = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)

    roll = random.uniform(0, 2*math.pi)
    pitch = random.uniform(0, 2*math.pi)
    yaw = random.uniform(0, 2*math.pi)
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    model_state.pose.orientation.x = quaternion[0]
    model_state.pose.orientation.y = quaternion[1]
    model_state.pose.orientation.z = quaternion[2]
    model_state.pose.orientation.w = quaternion[3]
    sms_req = SetModelStateRequest()
    sms_req.model_state.pose = model_state.pose
    sms_req.model_state.twist = model_state.twist
    sms_req.model_state.model_name = 'training_model'
    sms_req.model_state.reference_frame = 'world'
    set_model_state_prox(sms_req)

    return rospy.wait_for_message('/sensor_stick/point_cloud', PointCloud2)

def compute_color_histograms(cloud):
    # Compute histograms for the clusters

    colors_list = []

    for point in pc2.read_points(cloud, skip_nans=True):
        colors_list.append(float_to_rgb(point[3]))

    r_vals = []
    g_vals = []
    b_vals = []

    for color in colors_list:
        r_vals.append(color[0])
        g_vals.append(color[1])
        b_vals.append(color[2])

    rhist = np.histogram(r_vals, bins=32, range=(0, 256))
    ghist = np.histogram(g_vals, bins=32, range=(0, 256))
    bhist = np.histogram(b_vals, bins=32, range=(0, 256))

    #normalize the histograms
    rhist = ( 1.0*rhist[0]/np.sum(rhist[0]), rhist[1] )
    ghist = ( 1.0*ghist[0]/np.sum(ghist[0]), ghist[1] )
    bhist = ( 1.0*bhist[0]/np.sum(bhist[0]), bhist[1] )

    return (rhist, ghist, bhist)


def compute_normal_histograms(normal_cloud):
    norm_x_vals = []
    norm_y_vals = []
    norm_z_vals = []

    for norm_component in pc2.read_points(normal_cloud,
                                          field_names = ('normal_x', 'normal_y', 'normal_z'),
                                          skip_nans=True):
        norm_x_vals.append(norm_component[0])
        norm_y_vals.append(norm_component[1])
        norm_z_vals.append(norm_component[2])

    xhist = np.histogram(norm_x_vals, bins=32, range=(0, 1))
    yhist = np.histogram(norm_y_vals, bins=32, range=(0, 1))
    zhist = np.histogram(norm_z_vals, bins=32, range=(0, 1))

    #normalize the histograms
    xhist = ( 1.0*xhist[0]/np.sum(xhist[0]), xhist[1] )
    yhist = ( 1.0*yhist[0]/np.sum(yhist[0]), yhist[1] )
    zhist = ( 1.0*zhist[0]/np.sum(zhist[0]), zhist[1] )

    return (xhist, yhist, zhist)
        
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

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


if __name__ == '__main__':
    rospy.init_node('capture_node')
    rospy.wait_for_service('gazebo/get_model_state')
    rospy.wait_for_service('gazebo/set_model_state')
    rospy.wait_for_service('gazebo/get_physics_properties')
    rospy.wait_for_service('gazebo/set_physics_properties')
    rospy.wait_for_service('gazebo/spawn_sdf_model')

    models = [\
        'arm_part',
        'beer',
        'bowl',
        'create',
        'disk_part',
        'hammer',
        'plastic_cup',
        'soda_can']

    # Disable gravity and delete the ground plane
    intial_setup()

    labeled_features = []

    for model_name in models:
        spawn_model(model_name)

        for i in range(10):
            # make five attempts to get a valid a point cloud then give up
            sample_was_good = False
            try_count = 0
            while not sample_was_good and try_count < 5:
                sample_cloud = capture_sample()
                sample_cloud_arr = ros_to_pcl(sample_cloud).to_array()

                # Check for invalid clouds.
                if np.isnan(sample_cloud_arr).sum() > 0 or sample_cloud_arr.shape[0] == 0:
                    print('Invalid cloud detected')
                    try_count += 1
                else:
                    sample_was_good = True

            chists = compute_color_histograms(sample_cloud)

            normals = get_normals(sample_cloud)
            nhists = compute_normal_histograms(normals)
            feature = np.concatenate((
                                      chists[0][0],
                                      chists[1][0],
                                      chists[2][0],
                                      nhists[0][0],
                                      nhists[1][0],
                                      nhists[2][0]))
            labeled_features.append([feature, model_name])
            # if len(labeled_features) == 1:
            #     plot_histogram(nhists[0],nhists[1],nhists[2])
    
    pickle.dump(labeled_features, open('training_set.sav', 'wb'))
 
