cmake_minimum_required(VERSION 2.8.3)
project(opencv_bgknn)

find_package(Boost REQUIRED COMPONENTS
	python
)

find_package(PythonLibs REQUIRED)

find_package(OpenCV REQUIRED)

include_directories(
	${Boost_INCLUDE_DIRS}
	${PYTHON_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	)

add_library(opencv_bgknn SHARED bgfg_KNN.cpp)

target_link_libraries(opencv_bgknn 
	${Boost_LIBRARIES}
	${PYTHON_LIBRARIES}
	${OpenCV_LIBRARIES}
)
