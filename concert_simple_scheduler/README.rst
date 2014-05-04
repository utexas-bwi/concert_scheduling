Overview
========

Because different systems require different scheduling policies, the
`Robotics in Concert`_ (ROCON) framework allows multiple scheduler
implementations.  The scheduler is a ROS node running on same master
as the ROCON Conductor, services and other Solution components.

This `concert_simple_scheduler`_ ROS_ package implements a scheduler
node in Python.  It uses some infrastructure packages that other
scheduler implementations can also use or modify.

ROS interface
=============

simple_scheduler node
---------------------

This node provides a relatively simple fixed-priority scheduler which
allocates resources within each priority on a first-come, first-served
basis.

Subscribed topics
'''''''''''''''''

``rocon_scheduler`` (`scheduler_msgs/SchedulerRequests`_) 
    Scheduler requests.  Any ROCON service or application sending
    messages to the ``rocon_scheduler`` topic is called a
    **requester**.

``concert_client_changes`` (`concert_msgs/ConcertClients`_)
    ROCON clients known to the Conductor.


Published topics
''''''''''''''''

``rocon_scheduler_0123456789abcdef0123456789abcdef`` (`scheduler_msgs/SchedulerRequests`_)
    Per-requester scheduler feedback. Each **requester** assigns
    itself a `universally unique identifier`_ and subscribes to a
    feedback topic using the hexadecimal string representation of its
    UUID.

``resource_pool`` (`scheduler_msgs/KnownResources`_)
    The status of all ROCON clients currently managed by this scheduler.

Parameters
''''''''''

``~topic_name`` (string, default: rocon_scheduler)
    Common name prefix to use for the request and feedback
    topics. Since the feedback topic is generated dynamically, it
    would otherwise be difficult to remap.

Usage
'''''

    $ rosrun concert_simple_scheduler simple_scheduler

Protocol
========

Each `scheduler_msgs/SchedulerRequests`_ message describes all
resources currently desired by that requester.  The status of each
resource request is passed back and forth between the requester and
the scheduler via `scheduler_msgs/Request`_ elements contained in the
allocation and feedback topics.

Handling these messages and managing the resource request states can
be tricky, because state changes flow over the two topics
simultaneously.  Both schedulers and requesters should use the
`concert_scheduler_requests`_ package to perform the appropriate state
transitions for each request.

.. _`concert_msgs/ConcertClients`:
   https://github.com/robotics-in-concert/rocon_msgs/blob/hydro-devel/concert_msgs/msg/ConcertClients.msg
.. _`Robotics in Concert`: http://www.robotconcert.org/wiki/Main_Page
.. _`concert_scheduler_requests`: http://wiki.ros.org/concert_scheduler_requests
.. _`concert_simple_scheduler`: http://wiki.ros.org/concert_simple_scheduler
.. _ROS: http://wiki.ros.org
.. _`scheduler_msgs/KnownResources`:
   http://docs.ros.org/api/scheduler_msgs/html/msg/KnownResources.html
.. _`scheduler_msgs/Request`:
   http://docs.ros.org/api/scheduler_msgs/html/msg/Request.html
.. _`scheduler_msgs/SchedulerRequests`:
   http://docs.ros.org/api/scheduler_msgs/html/msg/SchedulerRequests.html
.. _`universally unique identifier`:
   http://en.wikipedia.org/wiki/Universally_unique_identifier
