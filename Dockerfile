FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

# Install basic dependencies (X11, OpenGL, utilities, Python)
RUN apt-get update && \
    apt-get install -y \
        xz-utils \
        git \
        python3 \
        python3-pip \
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
        sudo && \
    rm -rf /var/lib/apt/lists/*

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

# Add Gurobi env vars to user's bashrc for interactive sessions
RUN echo 'export GUROBI_HOME=/home/user/gurobi1203/linux64' >> ~/.bashrc && \
    echo 'export PATH="${PATH}:${GUROBI_HOME}/bin"' >> ~/.bashrc && \
    echo 'export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${GUROBI_HOME}/lib"' >> ~/.bashrc && \
    echo 'export GRB_LICENSE_FILE=/home/user/gurobi.lic' >> ~/.bashrc

WORKDIR /home/user/CoppeliaSim_Edu_V4_1_0_Ubuntu20_04

CMD ["./coppeliaSim.sh"]
