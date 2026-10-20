# NRDA Fleet OS: full simulation image (Gazebo Harmonic + ROS 2 Jazzy).
#   docker build -f docker/Dockerfile -t nrda .
#   docker run --rm -it --net=host -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --gpus all nrda ./nrda.sh sim
FROM osrf/ros:jazzy-desktop-full
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-gz ros-jazzy-nav2-map-server ros-jazzy-nav2-lifecycle-manager \
      python3-numpy python3-yaml python3-matplotlib python3-pytest xdg-utils \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /nrda
COPY . /nrda
RUN bash -c "source /opt/ros/jazzy/setup.bash && ./nrda.sh build"
CMD ["./nrda.sh", "sim"]
