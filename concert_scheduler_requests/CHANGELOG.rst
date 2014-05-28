Change history
==============

0.7.0 (2014-05-28)
------------------

 * Target version for Indigo Igloo release.
 * Rename `rocon_scheduler_requests`_ package to
   `concert_scheduler_requests`_, replacing the earlier version.


0.6.6 (2014-04-01)
------------------

 * Add thread locking for schedulers and requesters (`#24`_).
 * Rename ``ResourceReply`` class ``ActiveRequest`` (`#27`_).
 * Move ``resources`` module to another package (`#26`_).
 * Remove some *deprecated* methods: abort(), free(), reject(),
   release() (`#21`_)
 * Revise ``status`` labels and transitions.


0.6.5 (2013-12-19)
------------------

 * Experimental Python module for testing scheduler_msgs interface.

.. _concert_scheduler_requests: http://wiki.ros.org/concert_scheduler_requests
.. _rocon_scheduler_requests: http://wiki.ros.org/rocon_scheduler_requests

.. _`#21`: https://github.com/utexas-bwi/rocon_scheduler_requests/issues/21
.. _`#24`: https://github.com/utexas-bwi/rocon_scheduler_requests/issues/24
.. _`#26`: https://github.com/utexas-bwi/rocon_scheduler_requests/issues/26
.. _`#27`: https://github.com/utexas-bwi/rocon_scheduler_requests/issues/27
