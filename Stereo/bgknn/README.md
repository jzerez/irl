## Installation

### Get OpenCV Converter
git submodule init
git submodule update

## Build OpenCV Converter
cd numpy-opencv-converter
export CPLUS_INCLUDE_PATH=/usr/include/python2.7
mkdir -p build && cd build
cmake .. && make

## Build Background Subtractor KNN
cd ../..
mkdir -p build && cd build
cmake .. && make

## Copy Shared Object file to test directory
cp libopencv_bgknn.so ../test

## Test with Python
python main.py

## Test with C++
mkdir -p build && cd build
cmake .. && make
./test_bgknn
