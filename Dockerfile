FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN dpkg --add-architecture i386 && \
    apt-get update && \
    apt-get install -y \
      build-essential \
      jq \
      minicom \
      make \
      cmake \
      gdb-multiarch \
      automake \
      autoconf \
      libtool \
      libftdi-dev \
      libusb-1.0-0-dev \
      pkg-config \
      clang-format \
      git \
      wget \
      zip && \
    apt-get clean

WORKDIR /opt

RUN wget https://raw.githubusercontent.com/raspberrypi/pico-setup/master/pico_setup.sh

RUN sed -i 's/sudo[[:space:]]\+//g' pico_setup.sh

RUN chmod +x pico_setup.sh

RUN SKIP_VSCODE=1 SKIP_UART=1 bash ./pico_setup.sh

WORKDIR /vagrant

CMD ["/bin/bash"]
