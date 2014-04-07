Overview
========

The `concert_scheduler_requests`_ ROS_ package provides Python 
interfaces for managing scheduler requests within the `Robotics in
Concert`_ framework.

Because different systems require scheduling policies, the ROCON
design allows for multiple scheduler implementations.  This package
supplies a common infrastructure for various schedulers to use.

Scheduler Topics
----------------

The ROCON scheduler runs as a ROS node on same master as the ROCON
conductor, the rocon services and other Solution components.  It
subscribes to an allocation topic named **/rocon_scheduler** of type
`scheduler_msgs/SchedulerRequests`_.  Any ROCON service or application
sending messages to that topic is called a **requester**.  Each
requester assigns itself a `universally unique identifier`_ and
subscribes to a feedback topic using the hexadecimal string
representation of its UUID, in the form
**/rocon_scheduler_0123456789abcdef0123456789abcdef**. The scheduler
will provide status feedback on that topic via
`scheduler_msgs/SchedulerRequests`_ messages.

Scheduler Resource Requests
---------------------------

Each `scheduler_msgs/SchedulerRequests`_ message describes all
resources currently desired by that requester.  The status of each
resource request is passed back and forth between the requester and
the scheduler via `scheduler_msgs/Request`_ elements contained in the
allocation and feedback topics.

Handling these messages and managing the states of each resource
request can be quite tricky, because state changes flow over the two
topics simultaneously.  So, both schedulers and requesters need to
perform state transitions carefully and consistently for every
request.  

This package provides interfaces for schedulers and requesters to
perform those transitions correctly.

.. _`concert_scheduler_requests`: http://wiki.ros.org/concert_scheduler_requests
.. _`Robotics in Concert`: http://www.robotconcert.org/wiki/Main_Page
.. _ROS: http://wiki.ros.org
.. _`scheduler_msgs/Request`:
   http://docs.ros.org/api/scheduler_msgs/html/msg/Request.html
.. _`scheduler_msgs/SchedulerRequests`:
   http://docs.ros.org/api/scheduler_msgs/html/msg/SchedulerRequests.html
.. _`universally unique identifier`:
   http://en.wikipedia.org/wiki/Universally_unique_identifier
