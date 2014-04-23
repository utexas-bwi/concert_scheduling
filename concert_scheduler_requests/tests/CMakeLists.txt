### Unit tests
#
#   Only run when CATKIN_ENABLE_TESTING is true.

##  Python nose tests, not needing a running ROS core.
catkin_add_nosetests(test_common.py)
catkin_add_nosetests(test_priority_queue.py)
catkin_add_nosetests(test_transitions.py)

##  Unit tests using nose, but needing a running ROS core.
find_package(catkin REQUIRED COMPONENTS rostest)
add_rostest(py_example_requester.test)
add_rostest(py_example_scheduler.test)
add_rostest(py_timeout.test)
add_rostest(topic_param.test)
