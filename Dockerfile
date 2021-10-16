FROM subform/ros_with_cuda10.0:melodic

WORKDIR /root

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

#setting ros version
ENV ROS_DISTRO=melodic

COPY ./ros_entrypoint.sh /

RUN apt update && apt install -y vim curl \
        #for adding command catkin to install moveit
        python-catkin-tools \
        #installing realsense package
        ros-melodic-realsense2-camera \
        ros-melodic-rgbd-launch \
        #installing moveit package
        ros-melodic-moveit \
        #for installing motoman. this is one of dependency package
        ros-melodic-industrial-core

WORKDIR /root/ws

#prepare to build moveit package
WORKDIR /root/ws/src
RUN rosdep install -y --from-paths . --ignore-src --rosdistro melodic
WORKDIR /root/ws
RUN ["/bin/bash", "-c", "catkin config --extend /opt/ros/melodic --cmake-args -DCMAKE_BUILD_TYPE=Releasee"]

RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]