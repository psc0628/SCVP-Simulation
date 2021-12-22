# SCVP-Simulation
This is the simulation system of view planning for object reconstruction, supporting our paper "SCVP: Learning One-shot View Planning via Set Covering for Unknown Object Reconstruction". Our dataset can be accessed at www.kaggle.com/sicongpan/scvp-dataset. The source codes and pre-trained model are coming soon.
## Installion
For our c++ code, these libraries need to be installed: opencv 4.4.0, PCL 1.9.1, Eigen 3.3.9, OctoMap 1.9.6, Gurobi 9.1.1, Pytorch .
Thanks for NBV_net source codes (https://github.com/irvingvasquez/nbv-net).
Our codes can be run on both Windows 10 and Ubuntu 16.04. Please check for the name of source code folder.
## Note
Change "const static size_t maxSize = 100000;" to "const static size_t maxSize = 1000" in file OcTreeKey.h, so that the code will run faster.
## Usage
1. Sample 3d object model from *.obj or *.ply to *.pcd, and there are examples in our dataset. If you want to test your own model, please follow the sampling method (https://github.com/PointCloudLibrary/pcl/blob/master/tools/mesh_sampling.cpp), and run with "./pcl_mesh_sampling.exe *.ply *.pcd -n_samples 1000000 -leaf_size 0.5 -no_vis_result". Note that the -leaf_size is depending on the model, making sure that there are over 100000 points in cloud.
2. Change the directory and model name in file DefaultConfiguration.yaml.
3. Put the file DefaultConfiguration.yaml in the correct path, and then run compiled program of main.cpp.
4. For the choice of methods: MCMF is 0, OA is 1, UV is 2, RSE is 3, APORA is 4, Kr is 5, NBVNET is 6, SCVP is 7. If you want to run with NBV-Net or SCVP, please check nbv_net_path or sc_net_path in file DefaultConfiguration.yaml, and run both compiled program of main.cpp and "python nbv_net/run_test.py {}" or "python sc_net/run_test.py {}" (replace {} by your model name) in pytorch environment at the same time.
5. There is a parameter "show", by default is 1, which means that the middle cloud will be shown in a pcl window, and close it to continue. If you don't want to show the middle cloud, change it to 0.
## Questions
Please contact 18210240033@fudan.edu.cn or 4254145649@qq.com
