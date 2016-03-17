# rad_matrix


# To start

To start working, the V-REP simulator is required.

Copy the libv_repExt*.so files to the base directory of V-REP in order for the plugins to load.

Use the architect node to command all the simulations. 

# Limitations

Streaming the vision sensor decreases the framerate to around 15-20 fps. Thus if fast physics estimations are needed, do not use the video stream.
 
