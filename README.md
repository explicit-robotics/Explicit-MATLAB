# Explicit 
This repository contains **Exp[licit]** -- the MATLAB robotic simulator based on differential geometry. 

# Authors
This software was developed by Moses Chong-ook Nah and Johannes Lachner, working in the Newman Laboratory for Biomechanics and Human Rehabilitation at MIT.

# Details 
The documentation of the software will be deployed as a website in the near future.

## Comparison with Robotic Toolbox MATLAB
To run a comparison between our Exp[licit] and [Robotics Toolbox MATLAB](https://github.com/petercorke/robotics-toolbox-matlab) developed under [Prof. Peter Corke](https://github.com/petercorke), 
we have downloaded the [2nd edition: RTB10+MVTB4 (2017)](https://drive.matlab.com/sharing/e668b3b4-a452-464b-8e6e-77280e6cce21/), and saved the folder under `rvctools` directory. 
For details of the downloading process, please refer to [this website for details](https://petercorke.com/toolboxes/robotics-toolbox/).

To setup everything, you can simply run the setup.m file. This will add all necessary subfolders and add the RVC software to run the comparison with our software.

Type the following line and check whether the RVC software runs without error.
```
    mdl_panda
```