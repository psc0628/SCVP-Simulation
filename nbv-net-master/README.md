# nbv-net-master
Thanks for github.com/irvingvasquez/nbv-net.
This is the NBV-Net network under 32 candidate views.
## Installion
These libraries need to be installed: python 3.8.8, pytorch 1.8.1.
Our codes can be run on Windows 10.
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
