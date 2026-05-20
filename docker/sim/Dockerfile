# Fenrir — sim image (Phase 4)
#
# Builds on the BPC-PRP Phase 3 base and adds Gazebo Harmonic + ros_gz +
# the ROS 2 packages needed to spawn a robot from a URDF, bridge gz <-> ROS 2
# topics, and visualize the result.
#
# This image is what `ros2 launch fenrir_sim line.launch.py` (and the
# corridor / maze launch files later) actually runs inside. Students get
# it on demand:
#   docker build -t bpc-prp-sim:jazzy fenrir-project/docker/sim
#   docker run --rm -it --net=host ... bpc-prp-sim:jazzy \
#       ros2 launch fenrir_sim line.launch.py
#
# The build context expects bpc-prp-base:jazzy to be available locally OR
# overridden via --build-arg BASE_IMAGE to a published GHCR tag.

ARG BASE_IMAGE=bpc-prp-base:jazzy
FROM ${BASE_IMAGE} AS sim

USER root
ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=jazzy

# Gazebo Harmonic is in the standard apt repo for Ubuntu 24.04 starting
# late 2024; ros-jazzy-ros-gz pairs ros 2 jazzy <-> gz-harmonic.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-ros-gz \
        ros-${ROS_DISTRO}-ros-gz-bridge \
        ros-${ROS_DISTRO}-ros-gz-sim \
        ros-${ROS_DISTRO}-ros-gz-interfaces \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-robot-state-publisher \
        ros-${ROS_DISTRO}-joint-state-publisher \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-image-transport-plugins \
    && rm -rf /var/lib/apt/lists/*

# ---- gz environment ---------------------------------------------------------
# gz-sim looks for assets on these paths. We seed empty values so the launch
# files can append the simulation/worlds and simulation/description directories
# of the mounted workspace.
ENV GZ_SIM_RESOURCE_PATH=""
ENV GZ_SIM_SYSTEM_PLUGIN_PATH=""

# ---- developer convenience --------------------------------------------------
# Source gz_sim setup alongside ROS 2 in interactive shells.
RUN echo 'export GZ_VERSION=harmonic' >> /etc/bash.bashrc

USER prp
WORKDIR /workspace
CMD ["bash"]
