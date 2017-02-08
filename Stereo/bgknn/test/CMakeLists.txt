cmake_minimum_required(VERSION 2.8.3)
project(test_bgknn)

find_package(OpenCV REQUIRED)

link_directories(/home/jamiecho/Miscellaneous/bgknn/build/) # add library so

include_directories(
	${Boost_INCLUDE_DIRS}
	${PYTHON_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	)

add_executable(test_bgknn main.cpp)

target_link_libraries(test_bgknn 
	${Boost_LIBRARIES}
	${PYTHON_LIBRARIES}
	${OpenCV_LIBRARIES}
	opencv_bgknn
)

#target_link_libraries(test_bgknn
#	opencv_bgknn
#)
