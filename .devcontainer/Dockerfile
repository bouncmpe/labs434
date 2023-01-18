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

## Graphical apps (gtk) complain if we don't set these variables
ENV LANG="C"
ENV LC_ALL="C"

# Update pip and install Python packages
RUN python -m pip install --upgrade pip \
    && python -m pip install mujoco omegaconf jinja2 mediapy matplotlib black

RUN groupadd work -g 1000 \
   && useradd -ms /bin/bash bouncmpe -g 1000 -u 1000 \
   && printf "bouncmpe:bouncmpe" | chpasswd \
   && printf "bouncmpe ALL= NOPASSWD: ALL\\n" >> /etc/sudoers

USER bouncmpe

COPY models/ /mujoco/models/

ENV PATH=/home/bouncmpe/.local/bin:$PATH
