SHELL := /bin/bash

build: ./src
#	need to build darknet first
	catkin_make --only-pkg-with-deps darknet_ros
	catkin_make --only-pkg-with-deps motoman
	catkin_make --only-pkg-with-deps temptemp
	catkin_make -DCATKIN_WHITELIST_PACKAGES=""
	catkin_make
clean:
	rm -rf ./build ./devel