###########################################
# Xplore Control Station image Build Docker
###########################################
FROM ghcr.io/epflxplore/docker_commons:humble-desktop as build-cs

ARG DEBIAN_FRONTEND=noninteractive

# Install CS packages
RUN apt-get update && apt-get upgrade -y
RUN apt install -y curl

# Add USB rules
RUN echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
RUN /etc/init.d/udev restart

RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir evdev

RUN pip install --no-cache-dir channels==3.0.5 \
    pandas==2.0.3 \
    channels-redis==3.4.1 \
    transforms3d==0.4.1 \
    scikit-image \
    autobahn==22.7.1 \
    twisted \
