# pdm_sensor_fusion

Catkin package that enables to perform kalman filtering on incoming pose measurement (from vision) with a motion model. 
Different types of motion models can be created in main (src/filterer.cpp), the actual implementation of 1D KF class can be found in src/filter_classes.cpp . 

Note that the way the code is right now, only 1D KF is possible, but could be easily extendend to more dimensions. Also, in the current example, three models run separatels. (TO-DO: add time-stamp verification when no more messages come in or droupouts)

Place this repo in <my_catking_workspace>/src/pdm_sensor_fusion

see launch-files for example on how to use this catking package. 
