# RTIndy7 Task

This source is based on Ubuntu20.04, Xenomai-3.2.3, IgH EtherCAT Master

## Prerequisite
```
$ sudo apt-get install libeigen3-dev libpoco-dev libjsoncpp-dev
```

## build
```
$ mkdir build && cd build
$ cmake ../ && make -j($nproc)
```
### cmake options

All options are off as a default

- WITH_CASADI: Compute robot kinematics & dynamics based on Pinocchio library using CasADi symbolic framework
- WITH_POCO: Connect to remote visualizer such as blender, pybullet, etc.
- WITH_BULLET: Connect to pybullet server for visualization of simulation
- WITH_RP: Sellect robot model IndyRP, NRMK(default: Indy7)
- WITH_CB: Add I/O control box Ethercat slave 

```
$ sudo -s
$ ./RTIndy7_01
```

