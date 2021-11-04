#! /bin/sh

TOP_DIR=$(git rev-parse --show-toplevel)
CUR_DIR=$(pwd)
cd $TOP_DIR/build/

rm $TOP_DIR/build/* -rf
cd $TOP_DIR/build
cmake .. -G Ninja -DCMAKE_CXX_COMPILER=clang++
cmake --build .
cd $CUR_DIR
