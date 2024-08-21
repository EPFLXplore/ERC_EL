###########################################
# Xplore Electronics image
###########################################
FROM ghcr.io/epflxplore/docker_commons:foxy-jetson

ARG DEBIAN_FRONTEND=noninteractive

# Install ROS 2 Nav packages
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y --no-install-recommends
RUN apt-get -y install libyaml-cpp-dev iproute2 kmod busybox

# Add USB rules
RUN echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
RUN /etc/init.d/udev restart

USER $USERNAME

# Install Custom library

# Clone the library repository from GitHub
RUN git clone git@github.com:gbmhunter/CppLinuxSerial.git /home/$USERNAME/lib/CppLinuxSerial
# Create and set the working directory for the build
WORKDIR /home/$USERNAME/lib/CppLinuxSerial/build
# Configure and build the library using CMake
RUN cmake .. && make
# Optionally install the library system-wide
RUN sudo make install

# Set a diretory to store the project
WORKDIR /home/$USERNAME/dev_ws/src
COPY . .

# Set a directory to build the project
WORKDIR /home/$USERNAME/dev_ws

# Add the source of the project to the .bashrc
RUN echo "if [ -f /home/${USERNAME}/dev_ws/install/setup.bash ]; then source /home/${USERNAME}/dev_ws/install/setup.bash; fi" >> /home/${USERNAME}/.bashrc

RUN echo "alias avCode='cd /home/xplore/Desktop/EL_WS_2024/ERC_EL/ && code .'" >> /home/${USERNAME}/.bashrc
RUN echo "alias avLaunch='sh canfd.sh; sourcei; ros2 launch avionics_main avionics_launch.py'" >> /home/${USERNAME}/.bashrc
# RUN echo "alias avStart='./EL_start.sh'"" >> /home/${USERNAME}/.bashrc
# RUN echo "#Add colours and git branch to terminal \
#             parse_git_branch() { \
#                 git branch 2> /dev/null | sed -e '/^[^*]/d' -e 's/* \(.*\)/(\1)/' \
#             } \
#             if [ "$color_prompt" = yes ]; then \
#                 PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[01;31m\]$(parse_git_branch)\[\033[00m\]\$ ' \
#             else \
#                 PS1='${debian_chroot:+($debian_chroot)}\u@\h:\w$(parse_git_branch)\$ ' \
#             fi"  >> /home/${USERNAME}/.bashrc

# Clean up
RUN sudo rm -rf /var/lib/apt/lists/*

# Remove all the confidential Xplore source code from the image
RUN sudo rm -rf /home/$USERNAME/dev_ws/src/*
