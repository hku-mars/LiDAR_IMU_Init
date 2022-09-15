# Author: Tae Young Kim
# email: tyoung96@yonsei.ac.kr

FROM ros:melodic

# Install PCL & Eigen & essential libraries
RUN apt-get update && apt-get install -y cmake libeigen3-dev libpcl-dev \
    ros-melodic-rviz ros-melodic-pcl-ros ros-melodic-eigen-conversions \
    libatlas-base-dev libgoogle-glog-dev libsuitesparse-dev libglew-dev wget

# Install matplotlib
RUN apt-get update && apt-get install -y python-pip python-tk && pip install matplotlib

# Install ceres-solver
WORKDIR /home/
RUN wget https://github.com/ceres-solver/ceres-solver/archive/refs/tags/2.0.0.tar.gz
RUN tar zxf 2.0.0.tar.gz
RUN cd ceres-solver-2.0.0
RUN mkdir build && cd build
RUN ls
RUN cmake -DCMAKE_BUILD_TYPE=Release ./ceres-solver-2.0.0 && make -j2 && make install

# Install livox driver
WORKDIR /home/catkin_ws/src
RUN wget https://github.com/Livox-SDK/livox_ros_driver/archive/refs/tags/v2.6.0.tar.gz
RUN tar zxf v2.6.0.tar.gz && rm -rf v2.6.0.tar.gz

RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_init_workspace; cd .. && catkin_make'

# Load ROS environment at each run
COPY ./ros_entrypoint.sh /
RUN chmod 755 /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]