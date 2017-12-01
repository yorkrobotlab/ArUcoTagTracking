==========================
==                      ==
==  Tracking ARuCo tags ==
==                      ==
==========================

DATE: 15/11/2017

*** INTRODUCTION ***

In this package you'll find the code necessary to monitor the position 
the ARuCo tagson the arena. 

*** COMPILATION ***

To compile the controller, open up a shell, go to the directory where 
you unpacked the file and type:

$ mkdir build
$ cd build
$ cmake ..
$ ./machine_vision


*** TROUBLESHOOTING ***

If having troubles running experiments please check:
1. Make sure that the tracking server is not being used elsewhere.

*** FILE STRUCTURE ***

1. aruco_configuration.cpp - aruco_configuration.h
	- Functions related with configuring the ARuCO tracking system. In 
	addition retrieves the position and orientation of the tags.
2. machine_vision.cpp - machine_vision.h 
	- Main function logs the position of the ARuCO tags.

*** TIPS ***

Recording videos
mencoder mf://*.png -mf w=524:h=295:fps=30:type=png -ovc lavc -lavcopts vcodec=mpeg4:mbd=2:trell -oac copy -o ../output.avi
