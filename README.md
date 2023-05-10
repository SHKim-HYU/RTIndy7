# RTIndy7 Task

This source is built by cross compiler for Ubuntu14.04, 32bits


## build
```
$ mkdir build && cd build
$ cmake ../ && make

# Copy the release file to target PC
$ sudo scp ./RTIndy7_01 {target_USER}@{target_IP}:{target_Location}
