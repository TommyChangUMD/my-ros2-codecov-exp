#!/bin/bash
#
# This script should be invoked by "ros2 run" and after the unit test
# has been run.
#

set -ue                         # stop at the first error

PROG_DIR=$(dirname $(readlink -f "$0")) # where is the program located
EXEC_DIR=$(pwd -P)                      # where are we executing from
PROG_NAME=$(basename "$0")              # the program name without the path

# get ros package name
ROS_PACKAGE_NAME=$(basename $PROG_DIR)

# generat report info
BUILD_DIR=$EXEC_DIR/build/$ROS_PACKAGE_NAME/
rm -f $PROG_DIR/coverage.info
lcov --capture --directory $BUILD_DIR --output-file $PROG_DIR/coverage.info

# but, we want to filter out some files.
rm -f $PROG_DIR/coverage_cleaned.info
lcov --remove $PROG_DIR/coverage.info \
     '/opt/*' '/usr/*' 'rclcpp/*' \
     --output-file $PROG_DIR/coverage_cleaned.info


# finally generat report
rm -rf $EXEC_DIR/install/$ROS_PACKAGE_NAME/coverage/
genhtml $PROG_DIR/coverage_cleaned.info --output-directory \
        $EXEC_DIR/install/$ROS_PACKAGE_NAME/coverage
echo "Code Coverage generated:"
echo "     $PROG_DIR/coverage_cleaned.info"
echo "     $EXEC_DIR/install/$ROS_PACKAGE_NAME/coverage/index.html"


