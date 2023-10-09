# TagTracking (WIP)
Aruco tracking for steamvr

requires 2 cameras, some tags printed, and VMT installed

How to use: 

More in-depth tutorial here: (spanish) https://github.com/GoLez28/TagTracking/blob/master/tutorial/es.md

Calibrate the cameras by putting tag 0, 2, 4, 6, (can be changed in the config file) on the floor, the tag 0 will be the direction on steamvr, then press [1] to start calibrating (a value of less than 0.3 is good enough) (charuco board not supported yet), you can see the calibration in depth by pressing [7] to open the scene view window, pressing any key in it, will change what you see, being in "Raw trackers" let you see what the cameras see.   
Size of the trackers must be specified on the 'config.txt' file (trackerSize=[number]). Then adjust the offset of the room using the right controller by pressing [2] to toggle

parameters and shape of the trackers must be speciefied in 'trackers.txt', each tracker parameters layout is like following:
```
NAME  
id rotation offsetX offsetY offsetZ  
parameter=value
```

filterSmoothRot is how smooth it will turn  
filterSmoothPos is how smooth it will move

example in the files
