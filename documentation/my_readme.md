### Visualize CoM and Swing Foot Trajectory
Use `#define DRAW_DEBUG_PATH` and `DRAW_DEBUG_SWING`, for example, in `ConvexMPCLocomotion.cpp`
Visualization data structures are defined in `VisualizationData.h`
Visualization is implemented in `Graphics3D.cpp`

### Shortcur for pausing simulation
Press `V` on the active simulation window

### GCC version
The OS running on mini cheetah is Ubuntu 16.04. Make sure the GCC version is appropriate. I have tested GCC-4 and GCC-5. Both work. GCC-8 and higher did not work, where some standard library functions are renamed or missing. 