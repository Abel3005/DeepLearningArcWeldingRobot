FROM subform/ros_with_cuda10.0:melodic

WORKDIR /root

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

RUN apt update && apt install -y vim curl \
        #for adding command catkin to install moveit
        python-catkin-tools \
        #installing realsense package
        ros-melodic-realsense2-camera \
        #installing moveit package
        ros-melodic-moveit \
        #for installing motoman. this is one of dependency package
        ros-melodic-industrial-core

WORKDIR /root/workspace

#prepare to build moveit package
WORKDIR /root/workspace/src
RUN rosdep install -y --from-paths . --ignore-src --rosdistro melodic
WORKDIR /root/workspace
RUN ["/bin/bash", "-c", "catkin config --extend /opt/ros/melodic --cmake-args -DCMAKE_BUILD_TYPE=Releasee"]

RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]