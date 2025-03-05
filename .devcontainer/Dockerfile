FROM ubuntu:22.04

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
# - EGL (headless, hardware-accelerated)
# - GLFW (windowed, hardware-accelerated)
# - OSMesa (purely software-based)
#
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install \
        libgl1-mesa-glx \ 
        libosmesa6 \
        libglfw3 \
        libglew-dev \
    && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/* 

RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install \
        ffmpeg \
    && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/* 

# Update pip and install Python packages
RUN python -m pip install --upgrade pip \
    && python -m pip install --no-cache-dir \
        mujoco \
        dm_control \
        matplotlib 
        
## Graphical apps (gtk) complain if we don't set these variables
ENV LANG="C"
ENV LC_ALL="C"

# Add a container user (good practice)
ARG USERNAME=nonroot
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid ${USER_GID} $USERNAME &&\
    useradd -ms /bin/bash ${USERNAME} --uid $USER_UID --gid $USER_GID &&\
    echo "${USERNAME}:${USERNAME}" | chpasswd &&\
    echo "${USERNAME} ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/${USERNAME} &&\
    chmod 0440 /etc/sudoers.d/${USERNAME}

USER ${USERNAME}
WORKDIR /home/${USERNAME}

COPY models/ /mujoco/models/

ENV PATH=/home/${USERNAME}/.local/bin:$PATH
