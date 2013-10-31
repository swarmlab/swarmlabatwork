swarmlabatwork
==============

We used this code to win the robocup@work world championship 2013 in Eindhoven.

License
-------

We publish all of our code and documents under the [Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported (CC BY-NC-SA 3.0)](https://creativecommons.org/licenses/by-nc-sa/3.0).
For commercial interest please contact us at [swarmlab.unimaas.nl](http://swarmlab.unimaas.nl).


Usage
-----------

* Use the swarmlabatwork.rosinstall to install all packages.
* Use the install_ros_packages.sh script to install the dependencies.

* Patch the youbot_ros_pkg using the patch provided (this is needed to get voltage feedback in the joint_state messages)

Each test has its own package: e.g. slaw_bmt for the basic manipulation challenge, etc.
For each tests there are several launch files "xxx" stands for each tests:

* pre_xxx.launch (to be launched on the internal pc)
* pre_xxx_laptop.launch (to be launched on the external laptop)
* xxx.launch (to be launched either internally or externally) starts the test. 

Interesting packages
-------
* cust_dwa_local_planner: customised DWA planner used to navigate. Use at your own risk, it is best used with dual LIDAR setup (front and back facing), config in slaw_navigation.
* slaw_arm_navigation: Own IK calculation module and joint_trajectory_action with fail safe to recover after over voltages
* slaw_dashboard: pr2 like dashboard, for monitoring internal and external pc and batteries. slaw_diagnostics has to run.
* slaw_manipulation: object and hole detection
* slaw_smach: SMACH states which are used in each test

Troubleshooting
------------
If you have any questions or problems regarding robot setup or installation, feel free to contact us at [swarmlab.unimaas.nl](http://swarmlab.unimaas.nl).