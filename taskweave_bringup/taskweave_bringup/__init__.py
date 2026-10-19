"""Robot-agnostic bring-up of a taskweave cell: Nav2 and MoveIt from the robot pack, plus taskweave and the HMI.

Launch files: ``taskweave_bringup.launch.py`` (everything), ``taskweave_nav2.launch.py``,
``taskweave_moveit.launch.py``.
What belongs to a simulator or to hardware (clock handling, sensor bridges, the map's source) stays in the application.
"""
