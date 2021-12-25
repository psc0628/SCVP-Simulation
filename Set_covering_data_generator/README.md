# Set_covering_data_generator
Our dataset used in our paper can be accessed at www.kaggle.com/sicongpan/scvp-dataset.
These codes are for the generation of your own dataset.
DefaultConfiguration.yaml is the input config file of our code.
3d_models contians an example of input object 3D model (a pcd file sampled by a ply file).
SC_label_data contians an example of output labeled data (a gird file and a view label file).
points_on_sphere contians some condidate view space (thanks Spherical Codes neilsloane.com/packings).
SCVP_data_generation contians the source codes.
## Installion
These libraries need to be installed: opencv 4.4.0, PCL 1.9.1, Eigen 3.3.9, OctoMap 1.9.6, Gurobi 9.1.1.
Note that Gurobi is only free for academic use.
Our codes can be compiled by Visual Studio 2019 with c++ 14 and run on Windows 10. For other system, please check the file read/write or multithreading functions in the codes.
## Note
Change "const static size_t maxSize = 100000;" to "const static size_t maxSize = 1000" in file OcTreeKey.h, so that the code will run faster.
## Usage
1. Sample 3d object model from *.obj or *.ply to *.pcd and you can follow this sampling method https://github.com/PointCloudLibrary/pcl/blob/master/tools/mesh_sampling.cpp. The method can be run as "./pcl_mesh_sampling.exe *.ply *.pcd -n_samples 1000000 -leaf_size 0.5 -no_vis_result". Note that the -leaf_size is depending on the model, making sure that there are over 100000 points in cloud.
2. Change the model_path and name_of_pcd in file DefaultConfiguration.yaml.
3. Make sure that SCVP_data_generation (contains compiled program), points_on_sphere, DefaultConfiguration.yaml are in the same path.
4. Run compiled program of main.cpp, such as SCVP_data_generation/main.exe.
5. The output data can be found in SC_label_data folder.
