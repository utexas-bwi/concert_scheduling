### Unit tests
#
#   Only configured when CATKIN_ENABLE_TESTING is true.

# Unit tests using nose, but needing a running ROS core.
find_package(catkin REQUIRED COMPONENTS rostest)
add_rostest(simple_scheduler.test)
