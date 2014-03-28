Overview
========

The ``concert_simple_scheduler`` ROS_ package provides a Python
implementation of a scheduler node for managing scheduler requests
within the `Robotics in Concert`_ framework.

*This implementation is still experimental.*  

Because different systems require different scheduling policies, the
ROCON design allows for multiple scheduler implementations.  This
package supplies a simple fixed-priority scheduler and some common
infrastructure package that other scheduler implementations can use or
modify.

ROS simple_scheduler node
-------------------------

The ROCON scheduler runs as a ROS node on same master as the ROCON
conductor, the rocon services and other Solution components.

This implementation provides a relatively simple fixed-priority
scheduler which allocates resources within each priority on a
first-come, first-served basis.

Subscribed topics
'''''''''''''''''

``rocon_scheduler`` (`scheduler_msgs/SchedulerRequests`_) 
    Scheduler requests.  Any ROCON service or application sending
    messages to the ``rocon_scheduler`` topic is called a
    **requester**.

``concert_client_changes`` (`concert_msgs/ConcertClients`_)
    `Robotics in Concert`_ clients known to the Conductor.


Published topics
''''''''''''''''

``rocon_scheduler_0123456789abcdef0123456789abcdef`` (`scheduler_msgs/SchedulerRequests`_)
    Per-requester scheduler feedback. Each **requester** assigns
    itself a `universally unique identifier`_ and subscribes to a
    feedback topic using the hexadecimal string representation of its
    UUID.

``resource_pool`` (`scheduler_msgs/KnownResources`_)
    The status of all clients currently managed by this scheduler.

Protocol
''''''''

Each `scheduler_msgs/SchedulerRequests`_ message describes all
resources currently desired by that requester.  The status of each
resource request is passed back and forth between the requester and
the scheduler via `scheduler_msgs/Request`_ elements contained in the
allocation and feedback topics.

Handling these messages and managing the states of each resource
request can be quite tricky, because state changes flow over the two
topics simultaneously.  Both schedulers and requesters should use the
`rocon_scheduler_requests`_ package to perform the appropriate state
transitions for every request.

Usage
'''''

    $ rosrun concert_simple_scheduler simple_scheduler

.. _`concert_msgs/ConcertClients`:
   https://github.com/robotics-in-concert/rocon_msgs/blob/hydro-devel/concert_msgs/msg/ConcertClients.msg
.. _`Robotics in Concert`: http://www.robotconcert.org/wiki/Main_Page
.. _`rocon_scheduler_requests`: http://wiki.ros.org/rocon_scheduler_requests
.. _ROS: http://wiki.ros.org
.. _`scheduler_msgs/KnownResources`:
   https://github.com/robotics-in-concert/rocon_msgs/blob/hydro-devel/scheduler_msgs/msg/KnownResources.msg
.. _`scheduler_msgs/Request`:
   https://github.com/robotics-in-concert/rocon_msgs/blob/hydro-devel/scheduler_msgs/msg/Request.msg
.. _`scheduler_msgs/SchedulerRequests`:
   https://github.com/robotics-in-concert/rocon_msgs/blob/hydro-devel/scheduler_msgs/msg/SchedulerRequests.msg
.. _`universally unique identifier`:
   http://en.wikipedia.org/wiki/Universally_unique_identifier
