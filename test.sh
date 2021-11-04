TOP_DIR=$(git rev-parse --show-toplevel)
CUR_DIR=$(pwd)

cd $TOP_DIR/build/test
ctest .
cd $CUR_DIR
