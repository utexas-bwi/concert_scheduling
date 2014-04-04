cmake_minimum_required(VERSION 2.8.3)
project(concert_scheduler_requests)

# Minimal Python module setup
find_package(catkin REQUIRED COMPONENTS roslint)
catkin_python_setup()
catkin_package()

# Lint Python modules
file(GLOB_RECURSE ${PROJECT_NAME}_PY_SRCS 
     RELATIVE ${PROJECT_SOURCE_DIR} src/*.py)
roslint_python(${${PROJECT_NAME}_PY_SRCS}
               tests/example_requester.py
               tests/example_scheduler.py)

## Unit tests
if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()
