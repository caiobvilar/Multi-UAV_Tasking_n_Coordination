FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

# -------- Install basic dependencies --------
RUN apt-get update && \
    apt-get install -y \
        xz-utils \
        clang \
        clang-tools \
        llvm \
        gdb \
        build-essential \
        g++ \
        valgrind \
        kcachegrind \
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
        libxcb-glx0 \
        qt5-default \
        qtbase5-dev \
        qtbase5-dev-tools \
        libqt5gui5 \
        libdbus-1-3 \
        libavcodec-dev \
        libavformat-dev \
        libswscale-dev \
        ca-certificates \
        sudo \
        lsb-release \
        curl \
        gnupg2 \
        liblua5.1-0 \
        libqt5core5a libqt5gui5 libqt5widgets5 libqt5svg5 \
        libgl1-mesa-dev mesa-utils \
        libboost-program-options1.67.0 \
        libpcre2-16-0 \
        libdouble-conversion3 \
        libglib2.0-0 libgthread-2.0-0 \
        libx11-6 libxext6 libxcursor1 \
        libstdc++6 libgcc1 libz1 libdl2 \
        python3 python3-pip \
        locales && \
        locale-gen en_US.UTF-8 && \
        wget && \
    rm -rf /var/lib/apt/lists/*

# -------- Setup ROS Noetic repository --------
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

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

# -------- Install C++/Qt libraries for scientific/robotics tasks --------
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

# -------- Initialize rosdep --------
RUN rosdep init && \
    rosdep update

# -------- Create user with password and sudo access --------
RUN useradd -m -s /bin/bash user && \
    echo "user:user" | chpasswd && \
    usermod -aG sudo user

# -------- Copy and unpack CoppeliaSim, fix ownership, copy all plugins to app root --------
COPY CoppeliaSim_Edu_V4_1_0_Ubuntu20_04.tar.xz /home/user/
RUN cd /home/user && \
    tar -xJf CoppeliaSim_Edu_V4_1_0_Ubuntu20_04.tar.xz && \
    rm CoppeliaSim_Edu_V4_1_0_Ubuntu20_04.tar.xz && \
    chown -R user:user /home/user && \
    # Copy ALL plugin .so files to the app directory (to enable all features)
    for libdir in /home/user/CoppeliaSim_Edu_V4_1_0_Ubuntu20_04/programming/remoteApiBindings/lib/lib/*; do \
      for sofile in "$libdir"/*.so; do \
        cp "$sofile" /home/user/CoppeliaSim_Edu_V4_1_0_Ubuntu20_04/ 2>/dev/null || true; \
      done \
    done && \
    for sofile in /home/user/CoppeliaSim_Edu_V4_1_0_Ubuntu20_04/programming/remoteApiBindings/lib/*/*.so; do \
      cp "$sofile" /home/user/CoppeliaSim_Edu_V4_1_0_Ubuntu20_04/ 2>/dev/null || true; \
    done

# -------- Convenient symlink for CoppeliaSim --------
RUN ln -s /home/user/CoppeliaSim_Edu_V4_1_0_Ubuntu20_04/coppeliaSim.sh /usr/local/bin/coppeliaSim && \
    chmod +x /home/user/CoppeliaSim_Edu_V4_1_0_Ubuntu20_04/coppeliaSim.sh

# -------- Install Python 3.8, set as default, and scientific deps --------
RUN apt-get update && apt-get install -y software-properties-common && \
    add-apt-repository ppa:deadsnakes/ppa && apt-get update && \
    apt-get install -y python3.8 python3.8-dev python3.8-venv python3-pip && \
    update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 2

RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install numpy matplotlib pyzmq cbor cbor2

# -------- (Optional) Confirm Python/Gurobi setup --------
RUN python3 --version && python3 -m pip list

# -------- Copy/unpack Gurobi and license --------
COPY deps/gurobi12.0.3_linux64.tar.gz /home/user/
RUN cd /home/user && \
    tar -xzf gurobi12.0.3_linux64.tar.gz && \
    rm gurobi12.0.3_linux64.tar.gz && \
    chown -R user:user /home/user

COPY deps/gurobi.lic /home/user/gurobi.lic
RUN chown user:user /home/user/gurobi.lic

# -------- Set Gurobi env vars globally --------
ENV GUROBI_HOME=/home/user/gurobi1203/linux64
ENV PATH="${PATH}:${GUROBI_HOME}/bin"
ENV LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${GUROBI_HOME}/lib"
ENV GRB_LICENSE_FILE=/home/user/gurobi.lic

# -------- Install Gurobi Python interface --------
RUN python3 -m pip install --upgrade pip && \
    cd ${GUROBI_HOME} && \
    python3 -m pip install gurobipy

# -------- Switch to user workspace --------
WORKDIR /home/user
USER user

# -------- Add ROS and Gurobi env vars to bashrc --------
RUN echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc && \
    echo 'export GUROBI_HOME=/home/user/gurobi1203/linux64' >> ~/.bashrc && \
    echo 'export PATH="${PATH}:${GUROBI_HOME}/bin"' >> ~/.bashrc && \
    echo 'export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${GUROBI_HOME}/lib"' >> ~/.bashrc && \
    echo 'export PATH="$PATH:/home/user/.local/bin"' >> ~/.bashrc && \
    echo 'export GRB_LICENSE_FILE=/home/user/gurobi.lic' >> ~/.bashrc

# -------- Source ROS setup for this session --------
ENV ROS_DISTRO=noetic
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash"

CMD ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash"]
