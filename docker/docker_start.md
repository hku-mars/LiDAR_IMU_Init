## How to start LI-Init with Docker  

Author : [Taeyoung Kim](https://github.com/Taeyoung96) (tyoung96@yonsei.ac.kr)  

In `docker/` folder, there are some files related to Docker.  

This Dockerfile tested with Ubuntu 18.04, CUDA 11.2, NVIDIA Titan XP.  

### Make LI-Init Docker image

Move the terminal path to `/docker` and execute the following command.  

```
docker build -t li_init:1.0 .
```

After the image is created, you can execute `docker images` command to view the following results from the terminal.

```
REPOSITORY                  TAG                    IMAGE ID       CREATED             SIZE
li_init                     1.0                    ece4f57ca14b   48 minutes ago      7.94GB
```

### Make LI-Init Docker container  

When you create a docker container, you need several options to use the GUI and share folders.  

First, you should enter the command below in the local terminal to enable docker to communicate with Xserver on the host.  

```
xhost +local:docker
```

After that, make your own container with the command below.  

```
nvidia-docker run --privileged -it \
           -e NVIDIA_DRIVER_CAPABILITIES=all \
           -e NVIDIA_VISIBLE_DEVICES=all \
           --volume=${LiDAR_IMU_Init_repo_root}:/home/catkin_ws/src \
           --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
           --net=host \
           --ipc=host \
           --shm-size=1gb \
           --name=${docker container name} \
           --env="DISPLAY=$DISPLAY" \
           ${docker image} /bin/bash
```   

⚠️ **You should change {LiDAR_IMU_Init_repo_root}, {docker container name}, {docker image} to suit your environment.**  

For example,  
```
nvidia-docker run --privileged -it \
           -e NVIDIA_DRIVER_CAPABILITIES=all \
           -e NVIDIA_VISIBLE_DEVICES=all \
           --volume=/home/taeyoung/Desktop/LiDAR_IMU_Init:/home/catkin_ws/src \
           --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
           --net=host \
           --ipc=host \
           --shm-size=1gb \
           --name=li_init \
           --env="DISPLAY=$DISPLAY" \
           li_init:1.0 /bin/bash
```

If you have successfully created the docker container, the terminal output will be similar to the below.  

```
================Docker Env Ready================
root@taeyoung-cilab:/home/catkin_ws#
```  

### Launch LI-Init ROS package  

In your docker container, follow the commands.  

```
catkin_make
source devel/setup.bash
roslaunch lidar_imu_init xxx.launch
```

After initialization and refinement finished, the result would be written into `catkin_ws/src/LiDAR_IMU_Init/result/Initialization_result.txt`

---

These docker tutorial is tested on ubuntu 18.04 and may not be applied to arm platforms such as NVIDIA Jetson. In addition, this docker tutorial was used to execute the LI-Init with a bagfile, and if the actual sensor is used, it needs to be modified to create a docker container.  