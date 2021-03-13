#!/bin/bash

if [[ $# -eq 0 ]];  then
    echo "Using Default Config: Release"
    BUILD_TYPE=Release
else
	BUILD_TYPE=$1
fi

SRC_DIR=$(pwd)
CURR_DIR=$(pwd)
BUILD_PREFIX=build-$BUILD_TYPE
#INSTALL_DIR_BITCTRL=$CURR_DIR/install-all
INSTALL_DIR=$CURR_DIR/install-all

#BUILD_ITEM="server client"
BUILD_ITEM="client"

mkdir $BUILD_PREFIX
mkdir $INSTALL_DIR

sudo apt-get install qt5-default libeigen3-dev


if [[ $BUILD_ITEM == *"server"* ]]; then
rm -rf $BUILD_PREFIX/server
mkdir $BUILD_PREFIX/server
cd $BUILD_PREFIX/server
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE \
      -D ssnet_DIR=$INSTALL_DIR/lib/cmake/ssnet \
      -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR  \
      $SRC_DIR/BITServer/
make -j4
make install
cd $CURR_DIR
fi


if [[ $BUILD_ITEM == *"client"* ]]; then
rm -rf $BUILD_PREFIX/client
mkdir $BUILD_PREFIX/client
cd $BUILD_PREFIX/client
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE \
      -D ssnet_DIR=$INSTALL_DIR/lib/cmake/ssnet \
      -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR  \
      $SRC_DIR/BITClient/
make -j4
make install
cd $CURR_DIR
fi


