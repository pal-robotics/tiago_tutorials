FROM osrf/ros:melodic-desktop-full

LABEL maintainer="Yue Erro <yue.erro@pal-robotics.com>"

RUN mkdir -p /root/tiago_public_ws/src
WORKDIR /root/tiago_public_ws

RUN apt-get update && apt-get install -y \
    libv4l-dev \
    libv4l2rds0 \
    git \
    wget \
    vim \
    python3-vcstool \
    python-rosinstall \
    python-catkin-tools \
  && rm -rf /var/lib/apt/lists/* \
  && wget https://raw.githubusercontent.com/pal-robotics/tiago_tutorials/kinetic-devel/tiago_public-melodic.rosinstall \
  && vcs import src < tiago_public-melodic.rosinstall

ARG ROSDEP_IGNORE="opencv2 opencv2-nonfree pal_laser_filters speed_limit_node sensor_to_cloud hokuyo_node libdw-dev python-graphitesend-pip python-statsd pal_filters pal_vo_server pal_usb_utils pal_pcl pal_pcl_points_throttle_and_filter pal_karto pal_local_joint_control camera_calibration_files pal_startup_msgs pal-orbbec-openni2 dummy_actuators_manager pal_local_planner gravity_compensation_controller current_limit_controller dynamic_footprint dynamixel_cpp tf_lookup opencv3"

RUN apt-get update && rosdep install --from-paths src --ignore-src -y --rosdistro melodic --skip-keys="${ROSDEP_IGNORE}"

RUN bash -c "source /opt/ros/melodic/setup.bash \
    && catkin build -DCATKIN_ENABLE_TESTING=0 \
    && echo 'source /root/tiago_public_ws/devel/setup.bash' >> ~/.bashrc"

ENTRYPOINT ["bash"]
