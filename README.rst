concert_scheduling
==================

Scheduler support packages for the `Robotics in Concert`_ project.

*These packages are still in development.*  

Because different systems require different scheduling policies, the
ROCON design allows for multiple scheduler implementations.  These ROS
packages provide some common infrastructure, written in Python, for
various scheduler implementations to use or modify.

It also includes an example scheduler, which uses that infrastructure
to create a simple fixed-priority, first-come-first-served
implementation, probably the simplest scheduler one could actually
run.

Links to documentation:

 * `concert_resource_pool`_ interfaces for handling a pool of scheduler resources
 * `concert_scheduler_requests`_ interfaces for making and handling scheduler requests
 * `concert_simple_scheduler`_ a simple fixed-priority scheduler

.. _`concert_resource_pool`: http://wiki.ros.org/concert_resource_pool
.. _`concert_scheduler_requests`: http://wiki.ros.org/concert_scheduler_requests
.. _`concert_simple_scheduler`: http://wiki.ros.org/concert_simple_scheduler
.. _`Robotics in Concert`: http://www.robotconcert.org/wiki/Main_Page

