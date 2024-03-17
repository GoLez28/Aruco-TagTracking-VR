# TagTracking (WIP)
Aruco tag tracking for SteamVR  
(VRChat's OSC not supported)

requires 2 cameras (1 or more than 2 can also work), some tags printed, and VMT installed (https://github.com/gpsnmeajp/VirtualMotionTracker)

How to use: 

More in-depth tutorial here: (spanish) https://github.com/GoLez28/TagTracking/blob/master/tutorial/es.md  

Somethings i recommend is having the cameras at 960p (4:3 for better use of the camera, and i recommend 30 fps, 15 fps is also fine) at the minimum shutter speed posible, so motion blur is almost non-existent (increasing ISO helps), and having the tags be atleast 60 millimeters in width, so it can be tracked at 3 meters or so, being farther or/and having smaller tags may be innacurate and jittery, having less camera resolution may help with performance but tags must be bigger  

  
First you must calibrate the cameras, you can do so by pressing [4] and pointing the camera to the charuco image, once gathered enough samples press [3] again to finish  
Then you must calibrate the tracker by pressing [3] you must especify the tags id and lead, then having to move around the tracker as close to the camera posible, once done, add the generated file to trackers.txt  
Camera position can be estimated by pressing [1] and then pressing [2] while moving arround to add to the list, the tag 0 must be visible when finishing calibration by pressing [1] as this will be the direction on steamvr (charuco board not supported yet).  
You can see the calibration in depth by pressing [7] to open the scene view window, pressing any key in it, will change what you see, being in "Raw trackers" let you see what the cameras see, or "Tracker Calibration" to see the tracker progress in detail    
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

