# taskweave image: ROS 2 Jazzy, all six taskweave packages built from this checkout. Nothing is published.
#   docker build -t taskweave .
#   docker run --rm -v ./docs/examples/cell:/cell:ro taskweave taskweave check /cell
# See docs/install.md ("Container") and docker/compose.example.yaml.

ARG ROS_IMAGE=ros:jazzy-ros-base

# ---- build: dependencies of every package (build and run time) and colcon build ---------------------------------
FROM ${ROS_IMAGE} AS build
LABEL taskweave-t5=1
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends python3-colcon-common-extensions \
    && rosdep update --rosdistro "${ROS_DISTRO}" \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /ws
COPY . src/taskweave
RUN apt-get update \
    && rosdep install --from-paths src --ignore-src -y --rosdistro "${ROS_DISTRO}" \
    && rm -rf /var/lib/apt/lists/*
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
    && colcon build --install-base /opt/taskweave/install

# ---- runtime: run-time dependencies only, the install tree, the entrypoint ----------------------------------------
FROM ${ROS_IMAGE} AS runtime
LABEL taskweave-t5=1
ENV DEBIAN_FRONTEND=noninteractive
COPY --from=build /ws/src/taskweave /tmp/src/taskweave
RUN apt-get update \
    && rosdep update --rosdistro "${ROS_DISTRO}" \
    && rosdep install --from-paths /tmp/src --ignore-src -y --rosdistro "${ROS_DISTRO}" --dependency-types=exec \
    && rm -rf /var/lib/apt/lists/* /tmp/src /root/.ros/rosdep
COPY --from=build /opt/taskweave/install /opt/taskweave/install
COPY docker/entrypoint.sh /usr/local/bin/taskweave-entrypoint
RUN chmod +x /usr/local/bin/taskweave-entrypoint
# taskweave_core installs bin/taskweave (data_files); the install tree's bin/ goes on the PATH for every command
ENV PATH=/opt/taskweave/install/taskweave_core/bin:/opt/taskweave/install/bin:${PATH}
WORKDIR /cell
ENTRYPOINT ["taskweave-entrypoint"]
CMD ["taskweave", "--help"]
