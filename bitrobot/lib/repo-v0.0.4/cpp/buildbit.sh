#!/bin/bash


BUILD_TYPE=$1
SRC_DIR=$(pwd)
CURR_DIR=$(pwd)
BUILD_PREFIX=build-$BUILD_TYPE
INSTALL_DIR=$CURR_DIR/install-all
#BUILD_ITEM="geo ssnet bitctrl simu"
BUILD_ITEM="bitctrl"


mkdir $BUILD_PREFIX
mkdir $INSTALL_DIR

if [[ $BUILD_ITEM == *"geo"* ]]; then
rm -rf $BUILD_PREFIX/geo
mkdir $BUILD_PREFIX/geo
cd $BUILD_PREFIX/geo
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE \
      -D GEOGRAPHICLIB_LIB_TYPE=BOTH \
      -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR  \
      -D CMAKE_POSITION_INDEPENDENT_CODE:BOOL=true \
      $SRC_DIR/Bitctrl/GeographicLib-1.46 
make -j4
make install
cd $CURR_DIR
fi

if [[ $BUILD_ITEM == *"ssnet"* ]]; then
rm -rf $BUILD_PREFIX/ssnet
mkdir $BUILD_PREFIX/ssnet
cd $BUILD_PREFIX/ssnet
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE \
      -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR \
      -D CMAKE_POSITION_INDEPENDENT_CODE:BOOL=true \
      $SRC_DIR/ssnet/
make -j4
make install
cd $CURR_DIR
fi



if [[ $BUILD_ITEM == *"bitctrl"* ]]; then
rm -rf $BUILD_PREFIX/bitctrl
mkdir $BUILD_PREFIX/bitctrl
cd $BUILD_PREFIX/bitctrl
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE \
      -D GeographicLib_DIR=$INSTALL_DIR/lib/cmake/GeographicLib \
      -D ssnet_DIR=$INSTALL_DIR/lib/cmake/ssnet \
	  -D LibBitctrl_withNetwork:bool=true \
      -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR \
      $SRC_DIR/Bitctrl/
make -j4
make install
cd $CURR_DIR
fi


if [[ $BUILD_ITEM == *"simu"* ]]; then
rm -rf $BUILD_PREFIX/simu
mkdir $BUILD_PREFIX/simu
cd $BUILD_PREFIX/simu
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE \
      -D bitctrl_DIR=$INSTALL_DIR/lib/cmake/bitctrl \
      -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR/ \
      $SRC_DIR/simulation
make -j4
make install
cd $CURR_DIR
fi





