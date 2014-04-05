### Unit tests
#
#   Only configured when CATKIN_ENABLE_TESTING is true.

# Unit tests not needing a running ROS core.
catkin_add_nosetests(test_resource_pool.py)

# Unit tests using nose, but needing a running ROS core.
#find_package(catkin REQUIRED COMPONENTS rostest)
#add_rostest(simple_scheduler.test)
