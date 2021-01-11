I've done the receding horizon assuming that:

    - The trajectory considerated by the drone with the point-to-point path and the recalculated one with
    the receding horizon will not be exactly the same, but they are quite similar 

    - It's not necessary to pass through the points told by input, it's just a "reference" trajectory

    - The receding horizon path is obtained in function of steps, not in function of time (although 
    they can both be associated)

    - Sometimes, the script may not work because of the demanding trajectories. It will be necessary to 
    increase the RELAXATION_PARAMETER. This parameter can be found on the "formulation_RH.m" script, that
    is one of the algorithm parameters, at the end of the script.

    - To make this script work on your computer, you will need to change the script "addpaths.m" and put your
    own directories.
