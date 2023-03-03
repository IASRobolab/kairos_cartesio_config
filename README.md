kairos_cartesio_config
======================
Package with configuration files for the Kairos platform integrated with CartesI/O.

In particular the stack permits to control the UR manipulator and the base, constrained by joint limits, joint velocity limits, self-collision avoidance and tranformation from base to omni-wheels velocities.

Usage:
-----
```roslaunch kairos_cartesio_config cartesio.launch```

through the parameter ```use_capsules_collisions``` is possible to select between using convex-hulls of original meshes or capsules. 


[Download and install ```CartesI/O```](https://advrhumanoids.github.io/CartesianInterface/)
