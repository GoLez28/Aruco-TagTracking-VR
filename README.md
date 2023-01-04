# TagTracking (WIP)
Aruco tracking for steamvr

requires 2 cameras, some tags printed, and VMT installed

How to use: 

Calibrate the cameras by putting tracker 0, 2, 4, 6, all locking in the same direction (this will be the direction on steamvr) on the floor, and the press [1] (a value of less than 0.3 is good enough) (charuco board not supported yet), size of the trackers must be specified on the 'config.txt' file (trackerSize=[number]). Then adjust the offset of the room with the keys or manually using the right controller by pressing [2] to toggle

parameters and shape of the trackers must be speciefied in 'trackers.txt', each tracker parameters layout is like following:
```
NAME  
id rotation offsetX offsetY offsetZ  
parameter=value
```

filterSmoothRot is how smooth it will turn  
filterSmoothPos is how smooth it will move

example in the files
