FROM ubuntu:20.04

MAINTAINER Benny Sobol

RUN apt-get update
RUN apt-get install -y --no-install-recommends software-properties-common
RUN add-apt-repository ppa:rock-core/qt4
RUN add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"
RUN add-apt-repository main
RUN add-apt-repository universe
RUN add-apt-repository restricted
RUN add-apt-repository multiverse
RUN apt-get update --fix-missing
	
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y \
	build-essential cmake \
    wget git pkg-config \
    libjpeg-dev libtiff-dev \
    libjasper1 libjasper-dev \
    libpng-dev libavcodec-dev \
    libopenblas-dev liblapacke-dev \
    libavformat-dev libswscale-dev \
    libv4l-dev libgtk2.0-dev \
    libatlas-base-dev gfortran \
    freeglut3 freeglut3-dev \
    libtbb-dev libqt4-dev \
	libcgal-dev libpcl-dev \
	libgoogle-glog-dev libgflags-dev \
	libatlas-base-dev libeigen3-dev \
	libusb-1.0-0-dev \
    && apt-get -y clean all \
    && rm -rf /var/lib/apt/lists/*
	
RUN mkdir -p /home/user/benny
RUN mkdir -p /home/user/opencv
RUN mkdir -p /home/user/ceres

WORKDIR /home/user/opencv

RUN wget https://github.com/opencv/opencv_contrib/archive/4.4.0.tar.gz && \
    tar zxf 4.4.0.tar.gz && \
    rm 4.4.0.tar.gz && \
    mv opencv_contrib-4.4.0 opencv_contrib

RUN wget https://github.com/opencv/opencv/archive/4.4.0.tar.gz && \
    tar zxf 4.4.0.tar.gz && \
    rm 4.4.0.tar.gz && \
    mv opencv-4.4.0 opencv

WORKDIR /home/user/opencv/opencv
RUN cmake -Bbuild \
        -DCMAKE_BUILD_TYPE=RELEASE \
        -DOPENCV_EXTRA_MODULES_PATH=/home/user/opencv/opencv_contrib/modules \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
		-DBUILD_TESTS=OFF \
		-DBUILD_PERF_TESTS=OFF \
		-DBUILD_EXAMPLES=OFF \
		-DBUILD_DOCS=OFF \
		-DWITH_TBB=ON \
		-DWITH_OPENMP=ON \
		-DWITH_IPP=ON

RUN cd build \
    && make install \
    && cd /home/user/ \
    && rm -r opencv \
    && bash -c 'echo "/usr/local/lib" > /etc/ld.so.conf.d/opencv.conf' \
    && ldconfig
	
# ceres solver
WORKDIR /home/user/ceres
RUN git clone https://ceres-solver.googlesource.com/ceres-solver
WORKDIR /home/user/ceres/ceres-bin/
RUN cmake ../ceres-solver
RUN make -j4
RUN make install 


# Build and compile my code
WORKDIR /home/user/benny
RUN git clone https://github.com/BennySobol/Sr
RUN cmake build .
		
RUN cd build \
    && make
	