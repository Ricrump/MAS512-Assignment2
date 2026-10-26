# Q2 Lidar
1. Firstl run `ptcld_converter.py` to get the cropped Pointcloud
    - `swinging_load.pcap`and `swinging_load.json` must be in the same directory
### a) 
2. Run `cuboidRANSAC.py` to predict the center and pose of the swinging box
    - The resulting plots can be seen in `animation_frames`
### b)
2. Run `icpApprox.py` to predict the center and pose of the swinging box
3. Run `plotResults.py` to plot and compare the estimated trajectories of both procedures
    - Both trajectories can be seen in `trajectory_RANSAC_orange_ICP_blue.jpg` (RANSAC (Orange), ICP (blue))