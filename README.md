# TagTracking (WIP)
Aruco tag tracking for SteamVR  
(VRChat's OSC not supported)

requires 2 cameras (1 or more than 2 can also work), some tags printed, and VMT installed (https://github.com/gpsnmeajp/VirtualMotionTracker)

How to use: 

More in-depth tutorial here: (spanish) https://github.com/GoLez28/TagTracking/blob/master/tutorial/es.md  

Camera position can be estimated by pressing [1] and then pressing [2] while moving arround to add to the list, the tag 0 must be visible when finishing calibration by pressing [1] as this will be the direction on steamvr (charuco board not supported yet), you can see the calibration in depth by pressing [7] to open the scene view window, pressing any key in it, will change what you see, being in "Raw trackers" let you see what the cameras see.   
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

all the variables that config has: https://github.com/GoLez28/Aruco-TagTracking-VR/blob/master/tutorial/config_en.md


https://github.com/GoLez28/TagTracking/assets/15484340/968b5222-90e1-4f15-a05d-14d728cc2c7a

