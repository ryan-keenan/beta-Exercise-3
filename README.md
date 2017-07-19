# Object Recognition with ROS and PCL

In this exercise you will continue building up your perception pipeline in ROS.  Here, you are provided a gazebo world, where you can extract color and shape features from the objects that were sitting on the table from Exercise-1 and Exercise-2.

For the detailed steps of how to carry out this exercise, please see the [Object Recognition]() lesson in the RoboND classroom.  

## Setup
*  This exercise builds on what you've done in Exercises 1 and 2. If you're starting from scratch and you don't already have a `sensor_stick` folder in your `/src` directory, first copy/move the `sensor_stick` package to `/src` directory of your active ros workspace. If you have completed Exercises 1 and 2 and you already have a `sensor_stick` folder in the `/src` directory of your active ROS workspace, you can simply modify that folder by adding the following items from the `Exercise-3/sensor_stick` folder:

 * All additional scripts from the `scripts` folder
 * All additional launch files from the `launch` folder

* Make sure you have all the dependencies resolved by using the `rosdep install` tool and running `catkin_make`:  
 
```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
$ catkin_make
```

* If it's not already there, add the following lines to your `.bashrc` file  

```
export GAZEBO_MODEL_PATH=~/catkin_ws/src/sensor_stick/models
source ~/catkin_ws/devel/setup.bash
```

## Preparing for training

Launch the `training.launch` file to bring up the Gazebo environment: 

```sh
$ roslaunch sensor_stick training.launch
```
Next, in a new terminal window, run the `feature_extractor` node:

```sh
$ rosrun sensor_stick feature_extractor
```


## Capturing Features
Next, in a new terminal, run the `capture_features.py` script to capture and save features for each of the objects in the environment.  This script spawns each object in random orientations (default 10 orientations per object) and computes features based on the point clouds resulting from each of the random orientations.

```sh
$ rosrun sensor_stick capture_features.py
```

The features will now be captured and you can watch the objects being spawned in Gazebo. It should take 5-10 sec. for each random orientations (depending on your machine's resources) so with 8 objects total it takes awhile to complete. When it finishes running you should have a `training_set.sav` file.

## Training

Once your feature extraction has successfully completed, you're ready to train your model. First, however, if you don't already have them, you'll need to install the `sklearn`, `scipy` and `pandas` Python packages.  You can install these using `pip`:

```sh
pip install sklearn scipy pandas
```

After that, you're ready to run the `train_svm.py` model to train an SVM classifier on your labeled set of features.

```sh
$ rosrun sensor_stick train_svm.py
```

