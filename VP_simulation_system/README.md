# SCVP-Simulation
This is the view planning simulation system for object reconstruction.
DefaultConfiguration.yaml is the input config file of our code.
3d_models contians an example of input object 3D model (a pcd file sampled by a ply file).
HB26_example_SCVP contians an example of output results (point clouds, girds and indicators).
VP_simulation contians the source codes.
## Installion
These libraries need to be installed: opencv 4.4.0, PCL 1.9.1, Eigen 3.3.9, OctoMap 1.9.6.
Our codes can be compiled by Visual Studio 2019 with c++ 14 and run on Windows 10. For other system, please check the file read/write or multithreading functions in the codes.
## Note
Change "const static size_t maxSize = 100000;" to "const static size_t maxSize = 1000" in file OcTreeKey.h, so that the code will run faster.
## Usage
1. Sample 3d object model from *.obj or *.ply to *.pcd and you can follow this sampling method github.com/PointCloudLibrary/pcl/blob/master/tools/mesh_sampling.cpp. The method can be run as "./pcl_mesh_sampling.exe *.ply *.pcd -n_samples 1000000 -leaf_size 0.5 -no_vis_result". Note that the -leaf_size is depending on the model, making sure that there are over 100000 points in cloud.
2. Change the sc_net_path, nbv_net_path, model_path, name_of_pcd and method_of_IG in file DefaultConfiguration.yaml.
3. Make sure that VP_simulation (contains compiled program), DefaultConfiguration.yaml are in the same path. Make sure that sc-net/view_space.txt is available.
4. Run compiled program of main.cpp, such as VP_simulation/main.exe. For test NBV-Net or SCVP, run both main.exe and "python nbv_net/run_single_test.py {}" or "python sc_net/run_single_test.py {}" (replace {} by your model name) in pytorch environment at the same time.
6. For the choice of methods (method_of_IG), MCMF (https://github.com/psc0628/NBV-Simulation) is 0, OA is 1, UV is 2, RSE is 3, APORA is 4, Kr is 5, NBVNET is 6, SCVP is 7.
7. There is a parameter "show", by default is 1, which means that the middle cloud will be shown in a pcl window, and close it to continue. If you don't want to show the middle cloud, change it to 0.
