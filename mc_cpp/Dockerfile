FROM docker.io/ros:jazzy

ENV SHELL=/bin/bash
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DOMAIN_ID=1
ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH


# Install dependencies
RUN apt-get update && \
  apt-get install -y \
  python3-pip ros-jazzy-rviz2 ros-jazzy-rqt-common-plugins ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  libserial-dev \
  curl build-essential less htop tree nano vim neovim \
  && \
  rm -rf /var/cache/apk/* && \
  apt-get autoremove -y && \
  apt-get clean && \
  rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install --no-install-recommends -y\
        python3-numpy \
        python3-colcon-common-extensions \
        python3-matplotlib\
        python3-vcstool && \
    rm -rf /var/lib/apt/lists/*

# install additional externaly defined apt pkgs
COPY apt_requirements /apt_requirements
RUN apt-get update && \
    xargs apt-get install --no-install-recommends -y </apt_requirements && \
    rm -rf /var/lib/apt/lists/*

# install additional externaly defined pip pkgs
COPY python_requirements /python_requirements
RUN apt-get update && \
    xargs apt-get install --no-install-recommends -y </python_requirements && \
    rm -rf /var/lib/apt/lists/*

# Install python dependencies
RUN python3 -m pip install --no-cache-dir --break-system-packages \
  pysoem==1.1.12
  
# Clone SOEM (stable tag or master as needed)
RUN git clone --depth 1 https://github.com/OpenEtherCATsociety/SOEM.git /opt/SOEM \
  && mkdir -p /opt/SOEM/build && cd /opt/SOEM/build \
  && cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/usr/local \
  && cmake --build . -- -j$(nproc) \
  && cmake --build . --target install

# Setup environment
RUN \
  printf "echo rosbuild - Build all packages\n" >> ~/.bashrc && \
  printf "echo rossetup - Source ROS local setup variable\n" >> ~/.bashrc && \
  printf "echo rosclean - Delete build, install and log\n" >> ~/.bashrc && \
  printf "alias rossetup='cd /root/ws/ && source ~/ws/install/local_setup.bash && ros2 daemon start'\n" >> ~/.bashrc && \
  printf "alias rosbuild='colcon build --symlink-install'\n" >> ~/.bashrc && \
  printf "alias rosclean='rm -r ~/ws/build/ ~/ws/install/ ~/ws/log/'\n" >> ~/.bashrc && \
  printf "source /opt/ros/jazzy/setup.bash\n" >> ~/.bashrc && \
  printf "PS1='\[\e[32m\]rascl-container\[\e[0m\]:\[\e[34m\]\w\[\e[0m\]$ '\n" >> ~/.bashrc
  
RUN \
  apt -yq remove python3-matplotlib
RUN \
  pip install roboticstoolbox-python==1.1.1 --break-system-packages --ignore-installed filelock
RUN \
  pip install "numpy<2.0" --break-system-packages
RUN \
  pip install --break-system-packages websockets==11.0.3
RUN \  
  pip install matplotlib==3.7.5 --break-system-packages

WORKDIR /root/ws

CMD ["/bin/bash"]
