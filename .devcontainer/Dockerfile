FROM ubuntu:26.04

RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install \
        sudo \
        git \
        python3 \
        python3-pip \
        python-is-python3 \
    && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/* 

# Hardware rendering dependencies (ref: https://github.com/deepmind/dm_control)
#
# Support three different OpenGL rendering backends: 
# - EGL (headless, hardware-accelerated; needs the GLVND libEGL/libGL loaders)
# - GLFW (windowed, hardware-accelerated)
# - OSMesa (purely software-based)
#
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install \
        libegl1 \
        libgl1 \
        libopengl0 \
        libglx0 \
        libglvnd0 \
        libosmesa6 \
        libglfw3 \
        libglew-dev \
    && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/* 

RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install \
        ffmpeg \
    && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/* 

# Update pip and install Python packages
RUN python -m pip install --no-cache-dir --break-system-packages\
        mujoco \
        matplotlib 
        
## Graphical apps (gtk) complain if we don't set these variables
ENV LANG="C"
ENV LC_ALL="C"

# Add a container user (good practice)
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN echo "${USERNAME}:${USERNAME}" | chpasswd &&\
    echo "${USERNAME} ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/${USERNAME} &&\
    chmod 0440 /etc/sudoers.d/${USERNAME}

USER ${USERNAME}
WORKDIR /home/${USERNAME}

# COPY models/ /mujoco/models/

ENV PATH=/home/${USERNAME}/.local/bin:$PATH
ENV PIP_BREAK_SYSTEM_PACKAGES=1
