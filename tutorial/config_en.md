`float` means a decimal number, ej:`3.14159`  
`int` means a integer number, ej:`1337`  
`string` means a string of letters, ej:`sample text`  
`bool` means a boolean value, ej:`true` or `false`, only those
`list[]` values separated by commas ej:`list[int]`=`0,1,2,3`  


`trackerDelay=float` the drag that the tracker will havewhen the headset moves in milliseconds  
`poseAdjust=bool` some adjustments that the tracker will have to reduce errors, like blocking the leg rotation if its facing the wrong direction  
`poseAdjustLegDistance=float` leg distance from waist to foot  
`poseAdjustBackwardsAngle=float` vertical rotation of feet trackers  
`poseAdjustPanAngleRight=float` horizontal rotation of right foot tracker  
`poseAdjustPanAngleLeft=float` horizontal rotation of left foot tracker  
`poseAdjustWaist=string` the name of the tracker to use as waist  
`preNoiseReduction=int` tag noise reduction after is detected from camera (0=disabled, 1=enabled, 2=enabled w/o smooth rect)  
`postNoiseReduction=int` tracker noise redction before is sent to vmt/vrchat (0=disabled, 1=enabled, 2=partial(quicker))  
`enableIgnoreNoisyRotation=bool` if in pre-noise te rotation list is facing two directions equally, ignore if its more than three-quarters (barely noticeable)  
`oscAddress=string` ip adress of the osc client, usually 127.0.0.1   
`oscPort=int` port adress of the osc client, usually 39570 for VMT, and 9000 for VRChat   
`useVrchatOscTrackers=bool` send trackers to VRChat instead of VMT  
`sendHeadPositionVRC=bool` send head position to VRChat  
`sendHeadRotationVRC=bool` send head rotation to VRChat  
`roomOffsetX=float` offset the x-axis of the room  
`roomOffsetY=float` offset the y-axis of the room  
`roomOffsetZ=float` offset the z-axis of the room  
`trackerSize=int` size of the tags in millimeters  
`updatesPerSecond=int` application updates per seconds to calculate positions  
`interpolationTPS=int` updates per secon to interpolate  
`useInterpolation=bool` to interpolate between application updates  
`seekAhead=float` to try to guess when it will be next frame (0=no guessing, 1=guess next frame, 0.5=half interpolate/half preediction)  
`useSmoothCorners=bool` smoothening of the rects if it barely moves  
`cornersMaxDistance=int` max distance to start smoothening  
`cornersSmoothFactor=float` less of this value, more smooth the rects will be (from 0 to 1)  
`refineSearch=bool` modify the camera matrix after it calibrate the room, to reduce flickering  
`refineIterations=int` how many times will try to guess the best matrix  
`showThreadsTime=bool` show busy and idle time in milliseconds in the top bar  
`dynamicFiltering=bool` will reduce the post-noise reduction when the tracker moves  
`performanceUnderSample=float` will reduce frame resolution when performance mode is active (more noisy)  
`autoActivatePerformanceMode=bool` (UNUSED) will activate performance mode if cpu is struggling  
`perTrackerSize=list[int int]` a list of tracker that are different from trackerSize, set as `int int` `id size`, ej:`2 45,4 45,7 25`  
`tagsOnFloor=list[int]` a list of tags that will be on the floor when calibrating the room, dont bother changing it, it wont do much (barely noticeable)  
`totalCameras=int` the number of cameras that it will use, when this is changed you need to restart  

`cameraN...` which N is the camera id from 0 to totalCameras-1  
`cameraNQuality=int` the quality of camera, tag guessing will be more leaning to the higher quality camera  
`cameraNFile=int` the camera distortion parameter file path (as .xml)  
`cameraNWidth=int` camera width resolution  
`cameraNHeight=int` camera height resolution  
`cameraNResizeWidth=int` camera width to resize  
`cameraNResizeHeight=int` camera height to resize  
`cameraNIndex=int` camera to use, the camera index that the system will give  
`cameraNWorldResize=int` to reduce the distance that the tag was read  
`cameraNBrightness=int` to increase the brightness of the frame  
`cameraNFrameSkip=int`  the amount of frames to skip (to save cpu)
