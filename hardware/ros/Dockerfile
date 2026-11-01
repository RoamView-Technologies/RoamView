# syntax=docker/dockerfile:1

# -----------------------------------------------------------------------------
# ROS 2 Lyrical Luth development container. Two build targets:
#   base  general ROS 2 development: ros-base + colcon, rosdep, compilers (+ the desktop with INSTALL_DESKTOP=true)
#   sim   base + everything the packages in src/ list in their package.xml (for roamview_sim: Gazebo Jetty,
#         ros_gz, Nav2, robot_localization, RViz, teleop), installed by rosdep
# docker-compose.yml builds "base" for the ros2 service and "sim" for the sim service. See README.md.
# Base: the official "ros" Docker image, tag lyrical-ros-base
# (built on Ubuntu 26.04 "Resolute Raccoon", the Tier 1 platform for Lyrical)
# -----------------------------------------------------------------------------
ARG ROS_DISTRO=lyrical
FROM ros:${ROS_DISTRO}-ros-base AS base

ARG ROS_DISTRO=lyrical
# Set to "true" (via --build-arg) if you need RViz/demos/other GUI tools
ARG INSTALL_DESKTOP=false

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=${ROS_DISTRO} \
    LANG=en_US.UTF-8

# --- Development tooling: colcon, rosdep, compilers, editors ---------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-dev-tools \
        python3-colcon-common-extensions \
        python3-rosdep \
        build-essential \
        git \
        nano \
    && (rosdep init || true) \
    && rosdep update \
    && rm -rf /var/lib/apt/lists/*

# --- Optional: full desktop (RViz2, demos, GUI tools) -----------------------
RUN if [ "$INSTALL_DESKTOP" = "true" ]; then \
        apt-get update && \
        apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-desktop && \
        rm -rf /var/lib/apt/lists/* ; \
    fi

# --- Workspace ---------------------------------------------------------------
ENV ROS_WS=/root/ros2_ws
RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]


# -----------------------------------------------------------------------------
# manifests: picks the package.xml files out of src/ (a throwaway stage). The sim stage copies only these, so
# editing a launch file or a mesh never invalidates its big dependency layer; changing a package.xml does.
# -----------------------------------------------------------------------------
FROM base AS manifests
COPY src/ /tmp/src/
RUN mkdir -p /tmp/manifests \
    && cd /tmp/src \
    && find . -name package.xml -exec cp --parents {} /tmp/manifests/ \;


# -----------------------------------------------------------------------------
# sim: the RoamView simulation. The packages themselves are not copied in: docker-compose mounts src/ at
# /root/ros2_ws/src, so you edit them on the host and build them in the container with colcon.
# -----------------------------------------------------------------------------
FROM base AS sim

COPY --from=manifests /tmp/manifests /tmp/manifests
RUN apt-get update \
    && rosdep update --rosdistro ${ROS_DISTRO} \
    && rosdep install --from-paths /tmp/manifests --ignore-src -y --rosdistro ${ROS_DISTRO} \
    && apt-get install -y --no-install-recommends mesa-utils libxcb-cursor0 \
    && rm -rf /var/lib/apt/lists/* /tmp/manifests

# Every shell, including "docker exec -it roamview_sim bash", starts with ROS and the workspace sourced
RUN printf '%s\n' \
        "source /opt/ros/${ROS_DISTRO}/setup.bash" \
        '[ -f "${ROS_WS}/install/setup.bash" ] && source "${ROS_WS}/install/setup.bash"' >> /root/.bashrc

# Qt windows (Gazebo, RViz) draw through X11: RViz cannot draw on WSLg's Wayland.
# No .pyc files in the mounted src/ (they would belong to root on a Linux host).
ENV QT_QPA_PLATFORM=xcb \
    PYTHONDONTWRITEBYTECODE=1
