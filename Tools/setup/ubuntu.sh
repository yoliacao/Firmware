#! /usr/bin/env bash

## Bash script to setup PX4 development environment on Ubuntu LTS (18.04, 16.04).
## Can also be used in docker.
##
## Installs:
## - Common dependencies and tools for nuttx, jMAVSim, Gazebo
## - NuttX toolchain (omit with arg: --no-nuttx)
## - jMAVSim and Gazebo9 simulator (omit with arg: --no-sim-tools)
##
## Not Installs:
## - FastRTPS and FastCDR


INSTALL_NUTTX="true"
INSTALL_SIM="true"

# Parse arguments
for arg in "$@"
do
	if [[ $arg == "--no-nuttx" ]]; then
		INSTALL_NUTTX="false"
	fi

	if [[ $arg == "--no-sim-tools" ]]; then
		INSTALL_SIM="false"
	fi

done

# detect if running in docker
if [ -f /.dockerenv ]; then
	echo "Running within docker, installing initial dependencies";
	apt-get --quiet -y update && apt-get --quiet -y install \
		ca-certificates \
		curl \
		gnupg \
		gosu \
		lsb-core \
		sudo \
		wget \
		;
fi

# script directory
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

# check requirements.txt exists (script not run in source tree)
REQUIREMENTS_FILE="requirements.txt"
if [[ ! -f "${DIR}/${REQUIREMENTS_FILE}" ]]; then
	echo "FAILED: ${REQUIREMENTS_FILE} needed in same directory as ubuntu.sh (${DIR})."
	return 1
fi


# check ubuntu version
# instructions for 16.04, 18.04
# otherwise warn and point to docker?
UBUNTU_RELEASE=`lsb_release -rs`

if [[ "${UBUNTU_RELEASE}" == "14.04" ]]; then
	echo "Ubuntu 14.04 unsupported, see docker px4io/px4-dev-base"
	exit 1
elif [[ "${UBUNTU_RELEASE}" == "16.04" ]]; then
	echo "Ubuntu 16.04"
elif [[ "${UBUNTU_RELEASE}" == "18.04" ]]; then
	echo "Ubuntu 18.04"
fi



export DEBIAN_FRONTEND=noninteractive

echo
echo "Installing PX4 general dependencies"

# sudo apt-get update -yy --quiet
sudo apt-get -yy --quiet --no-install-recommends install \
	astyle \
	build-essential \
	ccache \
	clang \
	clang-tidy \
	cmake \
	cppcheck \
	doxygen \
	file \
	g++ \
	gcc \
	gdb \
	git \
	lcov \
	make \
	ninja-build \
	python3-pip \
	python3-pygments \
	python3-setuptools \
	python-pip \
	python-dev \
	rsync \
	shellcheck \
	unzip \
	wget \
	xsltproc \
	zip \
	;


if [[ "${UBUNTU_RELEASE}" == "16.04" ]]; then
	echo "Installing Ubuntu 16.04 PX4-compatible ccache version"
	wget -O /tmp/ccache_3.4.1-1_amd64.deb http://launchpadlibrarian.net/356662933/ccache_3.4.1-1_amd64.deb
	sudo dpkg -i /tmp/ccache_3.4.1-1_amd64.deb
fi

# Python3 dependencies
echo
echo "Installing PX4 Python3 dependencies"
sudo python3 -m pip install --upgrade pip setuptools wheel --proxy=http://127.0.0.1:7890
sudo python3 -m pip install -r ${DIR}/requirements.txt --proxy=http://127.0.0.1:7890


# Python2 dependencies
echo
echo "Installing PX4 Python2 dependencies"
sudo python2 -m pip install --upgrade pip setuptools wheel --proxy=http://127.0.0.1:7890
sudo python2 -m pip install -r ${DIR}/requirements.txt --proxy=http://127.0.0.1:7890


# NuttX toolchain (arm-none-eabi-gcc)
if [[ $INSTALL_NUTTX == "true" ]]; then

	echo
	echo "Installing NuttX dependencies"

	sudo apt-get -yy --quiet --no-install-recommends install \
		autoconf \
		automake \
		bison \
		bzip2 \
		flex \
		gdb-multiarch \
		gperf \
		libncurses-dev \
		libtool \
		pkg-config \
		vim-common \
		;

	# add user to dialout group (serial port access)
	sudo usermod -a -G dialout $USER

	# Remove modem manager (interferes with PX4 serial port/USB serial usage).
	sudo apt-get remove modemmanager -y

	# arm-none-eabi-gcc
	#NUTTX_GCC_VERSION="7-2017-q4-major"
	NUTTX_GCC_VERSION="7-2018-q2-6"
	GCC_VER_STR=$(arm-none-eabi-gcc --version)
	STATUSRETVAL=$(echo $GCC_VER_STR | grep -c "${NUTTX_GCC_VERSION}")

	if [ $STATUSRETVAL -eq "1" ]; then
		echo "arm-none-eabi-gcc-${NUTTX_GCC_VERSION} found, skipping installation"

	else
		echo "Installing arm-none-eabi-gcc-${NUTTX_GCC_VERSION}";
		wget -O /tmp/gcc-arm-none-eabi-${NUTTX_GCC_VERSION}-linux.tar.bz2 https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-rm/7-2017q4/gcc-arm-none-eabi-${NUTTX_GCC_VERSION}-linux.tar.bz2 && \
			sudo tar -jxf /tmp/gcc-arm-none-eabi-${NUTTX_GCC_VERSION}-linux.tar.bz2 -C /opt/;

		# add arm-none-eabi-gcc to user's PATH
		exportline="export PATH=/opt/gcc-arm-none-eabi-${NUTTX_GCC_VERSION}/bin:\$PATH"

		if grep -Fxq "$exportline" $HOME/.profile;
		then
			echo "${NUTTX_GCC_VERSION} path already set.";
		else
			echo $exportline >> $HOME/.profile;
		fi
	fi

fi

# Simulation tools
if [[ $INSTALL_SIM == "true" ]]; then

	echo
	echo "Installing PX4 simulation dependencies"

	# java (jmavsim or fastrtps)
	sudo apt-get -yy --quiet --no-install-recommends install \
		ant \
		openjdk-8-jre \
		openjdk-8-jdk \
		;

	# Gazebo
	sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
	wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
	# sudo apt-get update -yy --quiet
	sudo apt-get -yy --quiet --no-install-recommends install \
		gazebo9 \
		gstreamer1.0-plugins-bad \
		gstreamer1.0-plugins-base \
		gstreamer1.0-plugins-good \
		gstreamer1.0-plugins-ugly \
		libeigen3-dev \
		libgazebo9-dev \
		libgstreamer-plugins-base1.0-dev \
		libimage-exiftool-perl \
		libopencv-dev \
		libxml2-utils \
		pkg-config \
		protobuf-compiler \
		;

fi

if [[ $INSTALL_NUTTX == "true" ]]; then
	echo
	echo "Reboot or logout, login computer before attempting to build NuttX targets"
fi
