### Unit tests
#
#   Only run when CATKIN_ENABLE_TESTING is true.

##  Python

# Unit tests not needing a running ROS core.
catkin_add_nosetests(test_resource_pool.py)

# Unit tests using nose, but needing a running ROS core.
find_package(catkin REQUIRED COMPONENTS rostest)
#add_rostest(py_example_requester.test)
#add_rostest(py_example_scheduler.test)
#add_rostest(py_timeout.test)
