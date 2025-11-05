FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

# Install basic dependencies (X11, OpenGL, utilities, Python)
RUN apt-get update && \
    apt-get install -y \
        xz-utils \
        clang \
        clang-tools \
        llvm \
        build-essential \
        libstdc++-12-dev \
        clang-format \
        clang-tidy \
        git \
        python3 \
        python3-pip \
        python3-dev \
        libpython3.8-dev \
        libglib2.0-0 \
        libglu1-mesa \
        libxrender1 \
        libxcursor1 \
        libxrandr2 \
        libfreetype6 \
        libxinerama1 \
        libxi6 \
        libssl1.1 \
        libfontconfig1 \
        x11-apps \
        libxkbcommon-x11-0 \
        libxcb-icccm4 \
        libxcb-image0 \
        libxcb-keysyms1 \
        libxcb-render-util0 \
        libxcb-xinerama0 \
        libxcb-xinput0 \
        libxcb-shape0 \
        libxcb-shm0 \
        libxcb-xfixes0 \
        libxcb-randr0 \
        libxcb-util1 \
        libdbus-1-3 \
        libavcodec-dev \
        libavformat-dev \
        libswscale-dev \
        ca-certificates \
        sudo \
        lsb-release \
        curl \
        gnupg2 \
        wget && \
    rm -rf /var/lib/apt/lists/*

# Setup ROS Noetic repository
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Install ROS Noetic and required packages
RUN apt-get update && \
    apt-get install -y \
        ros-noetic-ros-base \
        ros-noetic-catkin \
        ros-noetic-roscpp \
        ros-noetic-rospy \
        ros-noetic-sensor-msgs \
        ros-noetic-std-msgs \
        ros-noetic-trajectory-msgs \
        ros-noetic-geometry-msgs \
        ros-noetic-tf2-ros \
        ros-noetic-tf2 \
        python3-rosdep \
        python3-rosinstall \
        python3-rosinstall-generator \
        python3-wstool \
        build-essential \
        cmake \
        gcc \
        g++ && \
    rm -rf /var/lib/apt/lists/*

# Install C++ libraries and Qt5 with all necessary modules
RUN apt-get update && \
    apt-get install -y \
        libarmadillo-dev \
        libboost-all-dev \
        libopencv-dev \
        qt5-default \
        libqt5charts5-dev \
        qtbase5-dev \
        qtchooser \
        qt5-qmake \
        qtbase5-dev-tools \
        libqt5printsupport5 \
        libqcustomplot-dev \
        qtdeclarative5-dev \
        qtquickcontrols2-5-dev \
        qml-module-qtquick2 \
        qml-module-qtquick-controls \
        qml-module-qtquick-controls2 \
        qml-module-qtquick-dialogs \
        qml-module-qtquick-layouts \
        qml-module-qtquick-window2 \
        libqt5quickcontrols2-5 && \
    rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && \
    rosdep update

# Create user with password and sudo access
RUN useradd -m -s /bin/bash user && \
    echo "user:user" | chpasswd && \
    usermod -aG sudo user

# Copy and unpack CoppeliaSim as root, then fix ownership
COPY CoppeliaSim_Edu_V4_1_0_Ubuntu20_04.tar.xz /home/user/
RUN cd /home/user && \
    tar -xJf CoppeliaSim_Edu_V4_1_0_Ubuntu20_04.tar.xz && \
    rm CoppeliaSim_Edu_V4_1_0_Ubuntu20_04.tar.xz && \
    chown -R user:user /home/user

# Create symlink for CoppeliaSim
RUN ln -s /home/user/CoppeliaSim_Edu_V4_1_0_Ubuntu20_04/coppeliaSim.sh /usr/local/bin/coppeliaSim && \
    chmod +x /home/user/CoppeliaSim_Edu_V4_1_0_Ubuntu20_04/coppeliaSim.sh

# Copy and unpack Gurobi as root
COPY deps/gurobi12.0.3_linux64.tar.gz /home/user/
RUN cd /home/user && \
    tar -xzf gurobi12.0.3_linux64.tar.gz && \
    rm gurobi12.0.3_linux64.tar.gz && \
    chown -R user:user /home/user

# Copy Gurobi license file
COPY deps/gurobi.lic /home/user/gurobi.lic
RUN chown user:user /home/user/gurobi.lic

# Set Gurobi environment variables globally
ENV GUROBI_HOME=/home/user/gurobi1203/linux64
ENV PATH="${PATH}:${GUROBI_HOME}/bin"
ENV LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${GUROBI_HOME}/lib"
ENV GRB_LICENSE_FILE=/home/user/gurobi.lic

# Install Gurobi Python interface
RUN python3 -m pip install --upgrade pip && \
    cd ${GUROBI_HOME} && \
    python3 -m pip install gurobipy

# Switch to user
WORKDIR /home/user
USER user

# Add ROS and Gurobi env vars to user's bashrc for interactive sessions
RUN echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc && \
    echo 'export GUROBI_HOME=/home/user/gurobi1203/linux64' >> ~/.bashrc && \
    echo 'export PATH="${PATH}:${GUROBI_HOME}/bin"' >> ~/.bashrc && \
    echo 'export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${GUROBI_HOME}/lib"' >> ~/.bashrc && \
    echo 'export GRB_LICENSE_FILE=/home/user/gurobi.lic' >> ~/.bashrc

# Source ROS setup for this session
ENV ROS_DISTRO=noetic
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash"

WORKDIR /home/user/CoppeliaSim_Edu_V4_1_0_Ubuntu20_04

CMD ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && ./coppeliaSim.sh"]
