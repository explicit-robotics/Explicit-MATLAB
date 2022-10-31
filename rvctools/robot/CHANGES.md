## General

- many functions factored out to SpatialMath toolbox
- fix bugs that prevented operation for 19b
- fixed `rtbdemo` demos that had atrophied
- Travis CI support now working
- Update the `README.md` file to include lots of animations produced by RTB
- improve unit tests, remove bugs, improve coverage
- move all mesh files to data/meshes
- replace `if/end` error tests with `assert`
- more LiveScript examples
- some changes to VREP interface

## In progress
- replace `rtbdemo` with AppDesigner version
- replace `DHFactor` java code with MATLAB code in `ETS.m`
- Reeds-Shepp + Dubbins planner need to be integrated with `Navigation` class
- Reeds-Shepp doesn't have all the cases
- URDF parser


## Simulink models

- add annotations to XY plots for all vehicle models, show initial/final pose
- added graphical gain sliders to some models
- tidyup of roblocks
- T2rpy rpy2T blocks have options for degrees and angle sequence

## Mobile robots
- animation option for `plot_vehicle` `ParticleFilter.run`
- `DXform` no longer requires MVTB, can used CVST or VLFEAT if installed

## Arm robots

- fixed torque function diagnostics for fdyn
- improvements to `ikine_sym`
- fixed model definiton for Franka-Emika PANDA `mdl_panda`
- fixed tile size calculation for robot plotting
- fixed bugs in `DHFactor`, better diagnostics
- `.twists` method gives joint screws