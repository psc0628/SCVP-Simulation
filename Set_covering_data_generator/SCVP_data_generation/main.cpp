#include <windows.h>
#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
typedef unsigned long long pop_t;

using namespace std;

#include "Share_Data.hpp"
#include "View_Space.hpp"
#include <gurobi_c++.h>

//Virtual_Perception_3D.hpp
void precept_thread_process(int i, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, octomap::point3d* _origin, octomap::point3d* _end, Eigen::Matrix4d* _view_pose_world, octomap::ColorOcTree* _ground_truth_model,Share_Data* share_data);

class Perception_3D {
public:
	Share_Data* share_data;
	octomap::ColorOcTree* ground_truth_model;
	int full_voxels;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

	Perception_3D(Share_Data* _share_data) {
		share_data = _share_data;
		ground_truth_model = share_data->ground_truth_model;
		full_voxels = share_data->full_voxels;
	}

	~Perception_3D() {
		
	}

	bool precept(View* now_best_view) { 
		double now_time = clock();
		//创建当前成像点云
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_parallel(new pcl::PointCloud<pcl::PointXYZRGB>);
		cloud_parallel->is_dense = false;
		cloud_parallel->points.resize(full_voxels);
		//获取视点位姿
		Eigen::Matrix4d view_pose_world;
		now_best_view->get_next_camera_pos(share_data->now_camera_pose_world, share_data->object_center_world);
		view_pose_world = (share_data->now_camera_pose_world * now_best_view->pose.inverse()).eval();
		//检查视点的key
		octomap::OcTreeKey key_origin;
		bool key_origin_have = ground_truth_model->coordToKeyChecked(now_best_view->init_pos(0), now_best_view->init_pos(1), now_best_view->init_pos(2), key_origin);
		if (key_origin_have) {
			octomap::point3d origin = ground_truth_model->keyToCoord(key_origin);
			//遍历每个体素
			octomap::point3d* end = new octomap::point3d[full_voxels];
			octomap::ColorOcTree::leaf_iterator it = ground_truth_model->begin_leafs();
			for (int i = 0; i < full_voxels; i++) {
				end[i] = it.getCoordinate();
				it++;
			}
			//ground_truth_model->write(share_data->save_path + "/test_camrea.ot");
			thread** precept_process = new thread * [full_voxels];
			for (int i = 0; i < full_voxels; i++) {
				precept_process[i] = new thread(precept_thread_process, i, cloud_parallel, &origin, &end[i], &view_pose_world, ground_truth_model, share_data);
			}
			for (int i = 0; i < full_voxels; i++)
				(*precept_process[i]).join();
			delete[] end;
			for (int i = 0; i < full_voxels; i++)
				precept_process[i]->~thread();
			delete[] precept_process;
		}
		else {
			cout << "View out of map.check." << endl;
		}
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
		cloud = temp;
		cloud->is_dense = false;
		cloud->points.resize(full_voxels);
		auto ptr = cloud->points.begin();
		int vaild_point = 0;
		auto p = cloud_parallel->points.begin();
		for (int i = 0; i < cloud_parallel->points.size(); i++, p++)
		{
			if ((*p).x == 0 && (*p).y == 0 && (*p).z == 0) continue;
			(*ptr).x = (*p).x;
			(*ptr).y = (*p).y;
			(*ptr).z = (*p).z;
			(*ptr).b = (*p).b;
			(*ptr).g = (*p).g;
			(*ptr).r = (*p).r;
			vaild_point++;
			ptr++;
		}
		cloud->width = vaild_point;
		cloud->height = 1;
		cloud->points.resize(vaild_point);
		//记录当前采集点云
		share_data->vaild_clouds++;
		share_data->clouds.push_back(cloud);
		//旋转至世界坐标系
		//*share_data->cloud_final += *cloud;
		cout << "virtual cloud get with executed time " << clock() - now_time << " ms." << endl;
		if (share_data->show) { //显示成像点云
			pcl::visualization::PCLVisualizer::Ptr viewer1(new pcl::visualization::PCLVisualizer("Camera"));
			viewer1->setBackgroundColor(255, 255, 255);
			viewer1->addCoordinateSystem(0.1);
			viewer1->initCameraParameters();
			viewer1->addPointCloud<pcl::PointXYZRGB>(cloud, "cloud");
			Eigen::Vector4d X(0.05, 0, 0, 1);
			Eigen::Vector4d Y(0, 0.05, 0, 1);
			Eigen::Vector4d Z(0, 0, 0.05, 1);
			Eigen::Vector4d O(0, 0, 0, 1);
			X = view_pose_world * X;
			Y = view_pose_world * Y;
			Z = view_pose_world * Z;
			O = view_pose_world * O;
			viewer1->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)), pcl::PointXYZ(X(0), X(1), X(2)), 255, 0, 0, "X" + to_string(-1));
			viewer1->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)), pcl::PointXYZ(Y(0), Y(1), Y(2)), 0, 255, 0, "Y" + to_string(-1));
			viewer1->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)), pcl::PointXYZ(Z(0), Z(1), Z(2)), 0, 0, 255, "Z" + to_string(-1));
			viewer1->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "X" + to_string(-1));
			viewer1->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "Y" + to_string(-1));
			viewer1->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "Z" + to_string(-1));
			while (!viewer1->wasStopped())
			{
				viewer1->spinOnce(100);
				boost::this_thread::sleep(boost::posix_time::microseconds(100000));
			}
		}

		cloud_parallel->~PointCloud();
		return true;
	}
};

inline octomap::point3d project_pixel_to_ray_end(int x, int y, rs2_intrinsics& color_intrinsics, Eigen::Matrix4d& now_camera_pose_world, float max_range) {
	float pixel[2] = { x ,y };
	float point[3];
	rs2_deproject_pixel_to_point(point, &color_intrinsics, pixel, max_range);
	Eigen::Vector4d point_world(point[0], point[1], point[2], 1);
	point_world = now_camera_pose_world * point_world;
	return octomap::point3d(point_world(0), point_world(1), point_world(2));
}

void precept_thread_process(int i, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, octomap::point3d* _origin, octomap::point3d* _end, Eigen::Matrix4d* _view_pose_world,octomap::ColorOcTree* _ground_truth_model, Share_Data* share_data) {
	//num++;
	octomap::point3d origin = *_origin;
	Eigen::Matrix4d view_pose_world = *_view_pose_world;
	octomap::ColorOcTree* ground_truth_model = _ground_truth_model;
	pcl::PointXYZRGB point;
	point.x = 0; point.y = 0; point.z = 0;
	//投影检测是否在成像范围内
	Eigen::Vector4d end_3d(_end->x(), _end->y(), _end->z(),1);
	Eigen::Vector4d vertex = view_pose_world.inverse() * end_3d;
	float point_3d[3] = { vertex(0), vertex(1),vertex(2) };
	float pixel[2];
	rs2_project_point_to_pixel(pixel, &share_data->color_intrinsics, point_3d);
	if (pixel[0] < 0 || pixel[0]>share_data->color_intrinsics.width || pixel[1] < 0 || pixel[1]>share_data->color_intrinsics.height) {
		cloud->points[i] = point;
		return;
	}
	//反向投影找到终点
	octomap::point3d end = project_pixel_to_ray_end(pixel[0], pixel[1], share_data->color_intrinsics, view_pose_world, 1.0);
	octomap::OcTreeKey key_end;
	octomap::point3d direction = end - origin;
	octomap::point3d end_point;
	//越过未知区域，找到终点
	bool found_end_point = ground_truth_model->castRay(origin, direction, end_point, true, 6.0 * share_data->predicted_size);
	if (!found_end_point) {//未找到终点，无观测数据
		cloud->points[i] = point;
		return;
	}
	if (end_point == origin) {
		cout << "view in the object. check!"<<endl;
		cloud->points[i] = point;
		return;
	}
	//检查一下末端是否在地图限制范围内
	bool key_end_have = ground_truth_model->coordToKeyChecked(end_point, key_end);
	if (key_end_have) {
		octomap::ColorOcTreeNode* node = ground_truth_model->search(key_end);
		if (node != NULL) {
			octomap::ColorOcTreeNode::Color color = node->getColor();
			point.x = end_point.x();
			point.y = end_point.y();
			point.z = end_point.z();
			point.b = color.b;
			point.g = color.g;
			point.r = color.r;
		}
	}
	cloud->points[i] = point;
}

//views_voxels_LM.hpp
class views_voxels_LM {
public:
	Share_Data* share_data;
	View_Space* view_space;
	vector<vector<bool>> graph;
	unordered_map<octomap::OcTreeKey, int, octomap::OcTreeKey::KeyHash>* voxel_id_map;	//体素下标
	int num_of_voxel;
	set<int>* chosen_views;
	GRBEnv* env;
	GRBModel* model;
	vector<GRBVar> x;
	GRBLinExpr obj;

	void solve() {
		// Optimize model
		model->optimize();
		// show nonzero variables
		/*for (int i = 0; i < share_data->num_of_views; i++)
			if (x[i].get(GRB_DoubleAttr_X) == 1.0)
				cout << x[i].get(GRB_StringAttr_VarName) << " " << x[i].get(GRB_DoubleAttr_X) << endl;
		// show num of views
		cout << "Obj: " << model->get(GRB_DoubleAttr_ObjVal) << endl;*/
	}

	vector<int> get_view_id_set() {
		vector<int> ans;
		for (int i = 0; i < share_data->num_of_views; i++)
			if (x[i].get(GRB_DoubleAttr_X) == 1.0) ans.push_back(i);
		return ans;
	}

	views_voxels_LM(Share_Data* _share_data, View_Space* _view_space, set<int>* _chosen_views) {
		double now_time = clock();
		share_data = _share_data;
		view_space = _view_space;
		chosen_views = _chosen_views;
		//建立体素的id表
		num_of_voxel = 0;
		voxel_id_map = new unordered_map<octomap::OcTreeKey, int, octomap::OcTreeKey::KeyHash>();
		for (int i = 0; i < share_data->voxels.size(); i++) {
			for (auto& it : *share_data->voxels[i]) {
				if (voxel_id_map->find(it.first) == voxel_id_map->end()) {
					(*voxel_id_map)[it.first] = num_of_voxel++;
				}
			}
		}
		//cout << num_of_voxel << " real | gt " << share_data->full_voxels << endl;
		graph.resize(num_of_voxel);
		for (int i = 0; i < share_data->num_of_views; i++) {
			graph[i].resize(num_of_voxel);
			for (int j = 0; j < num_of_voxel; j++) {
				graph[i][j] = 0;
			}
		}
		set<int> voxels_not_need;
		for (int i = 0; i < share_data->voxels.size(); i++) {
			for (auto& it : *share_data->voxels[i]) {
				graph[i][(*voxel_id_map)[it.first]] = 1;
				if (chosen_views->find(i) != chosen_views->end()) {
					voxels_not_need.insert((*voxel_id_map)[it.first]);
				}
			}
		}

		//cout << (*voxel_id_map).size() << endl;
		//cout << voxels_not_need.size() << endl;
		//octomap::ColorOcTree* octo_model = new octomap::ColorOcTree(share_data->octomap_resolution);
		//for (auto& it : *voxel_id_map) {
		//	if (voxels_not_need.find(it.second) == voxels_not_need.end()) continue;
		//	octo_model->setNodeValue(it.first, (float)0, true);
		//	octo_model->setNodeColor(it.first, 255, 0, 0);
		//}
		//octo_model->updateInnerOccupancy();
		//octo_model->write(share_data->save_path + "/Utest.ot");

		//建立对应的线性规划求解器
		now_time = clock();
		env = new GRBEnv();
		model = new GRBModel(*env);
		x.resize(share_data->num_of_views);
		// Create variables
		for (int i = 0; i < share_data->num_of_views; i++)
			x[i] = model->addVar(0.0, 1.0, 0.0, GRB_BINARY, "x" + to_string(i));
		// Set objective : \sum_{s\in S} x_s
		for (int i = 0; i < share_data->num_of_views; i++)
			obj += x[i];
		model->setObjective(obj, GRB_MINIMIZE);
		// Add linear constraint: \sum_{S:e\in S} x_s\geq1
		for (int j = 0; j < num_of_voxel; j++)
		{
			if (voxels_not_need.find(j) != voxels_not_need.end()) continue;
			GRBLinExpr subject_of_voxel;
			for (int i = 0; i < share_data->num_of_views; i++)
				if (graph[i][j] == 1) subject_of_voxel += x[i];
			model->addConstr(subject_of_voxel >= 1, "c" + to_string(j));
		}
		model->set("TimeLimit", "10");
		//cout << "Integer linear program formulated with executed time " << clock() - now_time << " ms." << endl;
	}

	~views_voxels_LM() {
		delete voxel_id_map;
		delete env;
		delete model;
	}
};

//SC_NBV_Labeler.hpp
class SC_NBV_Labeler 
{
public:
	Share_Data* share_data;
	View_Space* view_space;
	Perception_3D* percept;
	pcl::visualization::PCLVisualizer::Ptr viewer;
	int toward_state;
	int rotate_state;

	double check_size(double predicted_size, Eigen::Vector3d object_center_world, vector<Eigen::Vector3d>& points) {
		int vaild_points = 0;
		for (auto& ptr : points) {
			if (ptr(0) < object_center_world(0) - predicted_size || ptr(0) > object_center_world(0) + predicted_size) continue;
			if (ptr(1) < object_center_world(1) - predicted_size || ptr(1) > object_center_world(1) + predicted_size) continue;
			if (ptr(2) < object_center_world(2) - predicted_size || ptr(2) > object_center_world(2) + predicted_size) continue;
			vaild_points++;
		}
		return (double)vaild_points / (double)points.size();
	}

	SC_NBV_Labeler(Share_Data* _share_data, int _toward_state = 0, int _rotate_state = 0) {
		share_data = _share_data;
		toward_state = _toward_state;
		rotate_state = _rotate_state;
		//初始化GT
		//旋转6个朝向之一
		pcl::transformPointCloud(*share_data->cloud_pcd, *share_data->cloud_pcd, share_data->get_toward_pose(toward_state));
		//旋转8个角度之一
		Eigen::Matrix3d rotation;
		rotation = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
			Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd(45 * rotate_state * acos(-1.0) / 180.0, Eigen::Vector3d::UnitZ());
		Eigen::Matrix4d T_pose(Eigen::Matrix4d::Identity(4, 4));
		T_pose(0, 0) = rotation(0, 0); T_pose(0, 1) = rotation(0, 1); T_pose(0, 2) = rotation(0, 2); T_pose(0, 3) = 0;
		T_pose(1, 0) = rotation(1, 0); T_pose(1, 1) = rotation(1, 1); T_pose(1, 2) = rotation(1, 2); T_pose(1, 3) = 0;
		T_pose(2, 0) = rotation(2, 0); T_pose(2, 1) = rotation(2, 1); T_pose(2, 2) = rotation(2, 2); T_pose(2, 3) = 0;
		T_pose(3, 0) = 0;			   T_pose(3, 1) = 0;			  T_pose(3, 2) = 0;			     T_pose(3, 3) = 1;
		pcl::transformPointCloud(*share_data->cloud_pcd, *share_data->cloud_pcd, T_pose);
		//share_data->access_directory(share_data->save_path);
		//pcl::io::savePCDFile<pcl::PointXYZ>(share_data->save_path + "_" + to_string(toward_state) + "_" + to_string(rotate_state) + ".pcd", *share_data->cloud_pcd);
		//GT cloud
		share_data->cloud_ground_truth->is_dense = false;
		share_data->cloud_ground_truth->points.resize(share_data->cloud_pcd->points.size());
		share_data->cloud_ground_truth->width = share_data->cloud_pcd->points.size();
		share_data->cloud_ground_truth->height = 1;
		auto ptr = share_data->cloud_ground_truth->points.begin();
		auto p = share_data->cloud_pcd->points.begin();
		float unit = 1.0;
		for (auto& ptr : share_data->cloud_pcd->points) {
			if (fabs(ptr.x) >= 10 || fabs(ptr.y) >= 10 || fabs(ptr.z) >= 10) {
				unit = 0.001;
				cout << "change unit from <mm> to <m>." << endl;
				break;
			}
		}
		//检查物体大小，统一缩放为0.10m左右
		vector<Eigen::Vector3d> points;
		for (auto& ptr : share_data->cloud_pcd->points) {
			Eigen::Vector3d pt(ptr.x * unit, ptr.y * unit, ptr.z * unit);
			points.push_back(pt);
		}
		Eigen::Vector3d object_center_world = Eigen::Vector3d(0, 0, 0);
		//计算点云质心
		for (auto& ptr : points) {
			object_center_world(0) += ptr(0);
			object_center_world(1) += ptr(1);
			object_center_world(2) += ptr(2);
		}
		object_center_world(0) /= points.size();
		object_center_world(1) /= points.size();
		object_center_world(2) /= points.size();
		//二分查找BBX半径，以BBX内点的个数比率达到0.90-0.95为终止条件
		double l = 0, r = 0, mid;
		for (auto& ptr : points) {
			r = max(r, (object_center_world - ptr).norm());
		}
		mid = (l + r) / 2;
		double precent = check_size(mid, object_center_world, points);
		double pre_precent = precent;
		while (precent > 0.95 || precent < 1.0) {
			if (precent > 0.95) {
				r = mid;
			}
			else if (precent < 1.0) {
				l = mid;
			}
			mid = (l + r) / 2;
			precent = check_size(mid, object_center_world, points);
			if (fabs(pre_precent - precent) < 0.001) break;
			pre_precent = precent;
		}
		double predicted_size = 1.2 * mid;
		float scale = 1.0;
		if (predicted_size > 0.1) {
			scale = 0.1 / predicted_size;
			cout << "object large. change scale to about 0.1 m." << endl;
		}
		//转换点云
		//double min_z = 0;
		double min_z = object_center_world(2);
		for (int i = 0; i < share_data->cloud_pcd->points.size(); i++, p++)
		{
			(*ptr).x = (*p).x * scale * unit;
			(*ptr).y = (*p).y * scale * unit;
			(*ptr).z = (*p).z * scale * unit;
			(*ptr).b = 0;
			(*ptr).g = 0;
			(*ptr).r = 255;
			//GT插入点云
			octomap::OcTreeKey key;  bool key_have = share_data->ground_truth_model->coordToKeyChecked(octomap::point3d((*ptr).x, (*ptr).y, (*ptr).z), key);
			if (key_have) {
				octomap::ColorOcTreeNode* voxel = share_data->ground_truth_model->search(key);
				if (voxel == NULL) {
					share_data->ground_truth_model->setNodeValue(key, share_data->ground_truth_model->getProbHitLog(), true);
					share_data->ground_truth_model->integrateNodeColor(key, (*ptr).r, (*ptr).g, (*ptr).b);
				}
			}
			min_z = min(min_z, (double)(*ptr).z);
			//GT_sample插入点云
			octomap::OcTreeKey key_sp;  bool key_have_sp = share_data->GT_sample->coordToKeyChecked(octomap::point3d((*ptr).x, (*ptr).y, (*ptr).z), key_sp);
			if (key_have_sp) {
				octomap::ColorOcTreeNode* voxel_sp = share_data->GT_sample->search(key_sp);
				if (voxel_sp == NULL) {
					share_data->GT_sample->setNodeValue(key_sp, share_data->GT_sample->getProbHitLog(), true);
					share_data->GT_sample->integrateNodeColor(key_sp, (*ptr).r, (*ptr).g, (*ptr).b);
				}
			}
			ptr++;
		}
		//记录桌面
		share_data->min_z_table = min_z - share_data->ground_truth_resolution;
		
		share_data->ground_truth_model->updateInnerOccupancy();
		//share_data->ground_truth_model->write(share_data->save_path + "/GT.ot");
		//GT_sample_voxels
		//for (double x = share_data->object_center_world(0) - 0.2; x <= share_data->object_center_world(0) + 0.2; x += share_data->octomap_resolution - 0.001)
		//	for (double y = share_data->object_center_world(2) - 0.2; y <= share_data->object_center_world(2) + 0.2; y += share_data->octomap_resolution - 0.001) {
		//		double z = share_data->min_z_table;
		//		share_data->GT_sample->setNodeValue(x, y, z, share_data->GT_sample->getProbHitLog(), true);
		//		share_data->GT_sample->setNodeColor(x, y, z, 0, 0, 255);
		//	}
		share_data->GT_sample->updateInnerOccupancy();
		//share_data->GT_sample->write(share_data->save_path + "/GT_sample.ot");
		share_data->init_voxels = 0;
		for (octomap::ColorOcTree::leaf_iterator it = share_data->GT_sample->begin_leafs(), end = share_data->GT_sample->end_leafs(); it != end; ++it) {
			share_data->init_voxels++;
		}
		cout << "Map_GT_sample has voxels " << share_data->init_voxels << endl;
		//ofstream fout(share_data->save_path + "/GT_sample_voxels.txt");
		//fout << share_data->init_voxels << endl;

		share_data->full_voxels = 0;
		for (octomap::ColorOcTree::leaf_iterator it = share_data->ground_truth_model->begin_leafs(), end = share_data->ground_truth_model->end_leafs(); it != end; ++it) {
			share_data->full_voxels++;
		}

		//初始化viewspace
		view_space = new View_Space(share_data);

		//相机类初始化
		percept = new Perception_3D(share_data);
		
		srand(time(0));
	}

	int label() {
		double now_time = clock();
		for (int i = 0; i < view_space->views.size(); i++) {
			percept->precept(&view_space->views[i]);
			//get voxel map
			int num = 0;
			unordered_map<octomap::OcTreeKey, int, octomap::OcTreeKey::KeyHash>* voxel = new unordered_map<octomap::OcTreeKey, int, octomap::OcTreeKey::KeyHash>();
			for (int j = 0; j < share_data->clouds[i]->points.size(); j++) {
				octomap::OcTreeKey key = share_data->octo_model->coordToKey(share_data->clouds[i]->points[j].x, share_data->clouds[i]->points[j].y, share_data->clouds[i]->points[j].z);
				if (voxel->find(key) == voxel->end()) {
					(*voxel)[key] = num++;
				}
			}
			share_data->voxels.push_back(voxel);
		}
		cout << "all virtual cloud get with executed time " << clock() - now_time << " ms." << endl;

		/*//show covering
		set<int> chosen_views;
		views_voxels_LM* SCOP_solver = new views_voxels_LM(share_data, view_space, &chosen_views);
		SCOP_solver->solve();
		vector<int> need_views = SCOP_solver->get_view_id_set();
		delete SCOP_solver;
		pcl::visualization::PCLVisualizer::Ptr viewer1(new pcl::visualization::PCLVisualizer("Label"));
		viewer1->setBackgroundColor(255, 255, 255);
		//viewer1->addCoordinateSystem(0.1);
		viewer1->initCameraParameters();
		//红 绿 蓝 黄 洋红 青 黑 紫
		int r[8] = {255,0,0,255,255,0,128};
		int g[8] = {0,255,0,255,0,255,0};
		int b[8] = {0,0,255,0,255,255,255};
		for (int j = 0; j < need_views.size(); j++) { // 2 3 9 13 23 27 30 31
			cout << "view id is " << need_views[j] << "." << endl;
			//int r = rand() % 256;
			//int g = rand() % 256;
			//int b = rand() % 256;
			cout << r[j] << " " << g[j] << " " << b[j] << endl;
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(share_data->clouds[need_views[j]], r[j], g[j], b[j]);
			//pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZRGB> RandomColor(share_data->clouds[need_views[j]]);
			viewer1->addPointCloud<pcl::PointXYZRGB>(share_data->clouds[need_views[j]], single_color, "cloud" + to_string(need_views[j]));
			Eigen::Vector4d X(0.05, 0, 0, 1);
			Eigen::Vector4d Y(0, 0.05, 0, 1);
			Eigen::Vector4d Z(0, 0, 0.05, 1);
			Eigen::Vector4d O(0, 0, 0, 1);
			view_space->views[need_views[j]].get_next_camera_pos(view_space->now_camera_pose_world, view_space->object_center_world);
			Eigen::Matrix4d view_pose_world = (view_space->now_camera_pose_world * view_space->views[need_views[j]].pose.inverse()).eval();
			X = view_pose_world * X;
			Y = view_pose_world * Y;
			Z = view_pose_world * Z;
			O = view_pose_world * O;
			viewer1->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)), pcl::PointXYZ(X(0), X(1), X(2)), r[j], g[j], b[j], "X" + to_string(j));
			viewer1->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)), pcl::PointXYZ(Y(0), Y(1), Y(2)), r[j], g[j], b[j], "Y" + to_string(j));
			viewer1->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)), pcl::PointXYZ(Z(0), Z(1), Z(2)), r[j], g[j], b[j], "Z" + to_string(j));
			viewer1->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "X" + to_string(j));
			viewer1->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "Y" + to_string(j));
			viewer1->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "Z" + to_string(j));
		}
		//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color_black(share_data->clouds[i], 0, 0, 0);
		//viewer1->addPointCloud<pcl::PointXYZRGB>(share_data->clouds[i], color_black, "cloud" + to_string(i));
		while (!viewer1->wasStopped())
		{
			viewer1->spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}*/

		now_time = clock();
		for (int i = 0; i < view_space->views.size(); i++) if(i==0)
		{
			set<int> chosen_views;
			chosen_views.insert(i);
			views_voxels_LM* SCOP_solver = new views_voxels_LM(share_data, view_space, &chosen_views);
			SCOP_solver->solve();
			vector<int> need_views =  SCOP_solver->get_view_id_set();
			delete SCOP_solver;

			octomap::ColorOcTree* octo_model_test = new octomap::ColorOcTree(share_data->octomap_resolution);
			for (auto p : share_data->clouds[i]->points) {
				octo_model_test->setNodeValue(p.x, p.y, p.z, octo_model_test->getProbHitLog(), true);
				octo_model_test->integrateNodeColor(p.x, p.y, p.z, 255, 0, 0);
			}
			for (double x = share_data->object_center_world(0) - 0.2; x <= share_data->object_center_world(0) + 0.2; x += share_data->octomap_resolution)
				for (double y = share_data->object_center_world(2) - 0.2; y <= share_data->object_center_world(2) + 0.2; y += share_data->octomap_resolution) {
					double z = share_data->min_z_table;
					octo_model_test->setNodeValue(x, y, z, octo_model_test->getProbHitLog(), true);
					octo_model_test->integrateNodeColor(x, y, z, 0, 0, 255);
				}
			octo_model_test->updateInnerOccupancy();
			//octo_model_test->write(share_data->save_path + "/test.ot");
			int num_of_test = 0;
			for (octomap::ColorOcTree::leaf_iterator it = octo_model_test->begin_leafs(), end = octo_model_test->end_leafs(); it != end; ++it) {
				num_of_test++;
			}
			//cout << num_of_test << " " << share_data->clouds[i]->points.size() << endl;
			Perception_3D test(share_data);
			test.ground_truth_model = octo_model_test;
			test.full_voxels = num_of_test;
			test.precept(&view_space->views[i]);
			delete octo_model_test;
			/*
			Eigen::Matrix4d view_pose_world;
			view_space->views[i].get_next_camera_pos(share_data->now_camera_pose_world, share_data->object_center_world);
			view_pose_world = (share_data->now_camera_pose_world * view_space->views[i].pose.inverse()).eval();
			pcl::visualization::PCLVisualizer::Ptr viewer1(new pcl::visualization::PCLVisualizer("Camera"));
			viewer1->setBackgroundColor(0, 0, 0);
			viewer1->addCoordinateSystem(0.1);
			viewer1->initCameraParameters();
			viewer1->addPointCloud<pcl::PointXYZRGB>(share_data->clouds[view_space->views.size()], "cloud");
			Eigen::Vector4d X(0.05, 0, 0, 1);
			Eigen::Vector4d Y(0, 0.05, 0, 1);
			Eigen::Vector4d Z(0, 0, 0.05, 1);
			Eigen::Vector4d O(0, 0, 0, 1);
			X = view_pose_world * X;
			Y = view_pose_world * Y;
			Z = view_pose_world * Z;
			O = view_pose_world * O;
			viewer1->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)), pcl::PointXYZ(X(0), X(1), X(2)), 255, 0, 0, "X" + to_string(-1));
			viewer1->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)), pcl::PointXYZ(Y(0), Y(1), Y(2)), 0, 255, 0, "Y" + to_string(-1));
			viewer1->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)), pcl::PointXYZ(Z(0), Z(1), Z(2)), 0, 0, 255, "Z" + to_string(-1));
			while (!viewer1->wasStopped())
			{
				viewer1->spinOnce(100);
				boost::this_thread::sleep(boost::posix_time::microseconds(100000));
			}
			*/

			octomap::ColorOcTree* octo_model = new octomap::ColorOcTree(share_data->octomap_resolution);
			for (double x = share_data->object_center_world(0) - share_data->predicted_size; x <= share_data->object_center_world(0) + share_data->predicted_size; x += share_data->octomap_resolution)
				for (double y = share_data->object_center_world(1) - share_data->predicted_size; y <= share_data->object_center_world(1) + share_data->predicted_size; y += share_data->octomap_resolution)
					for (double z = share_data->object_center_world(2) - share_data->predicted_size; z <= share_data->object_center_world(2) + share_data->predicted_size; z += share_data->octomap_resolution)
						octo_model->setNodeValue(x, y, z, (float)0, true); //初始化概率0.5，即logodds为0
			octo_model->updateInnerOccupancy();
			octomap::Pointcloud cloud_octo;
			for (auto p : share_data->clouds[view_space->views.size()]->points) {
				cloud_octo.push_back(p.x, p.y, p.z);
			}
			octo_model->insertPointCloud(cloud_octo, octomap::point3d(view_space->views[i].init_pos(0), view_space->views[i].init_pos(1), view_space->views[i].init_pos(2)), -1, true, false);
			for (auto p : share_data->clouds[i]->points) {
				if (p.z >= share_data->min_z_table + share_data->octomap_resolution) octo_model->integrateNodeColor(p.x, p.y, p.z, 255, 0, 0);
				else octo_model->integrateNodeColor(p.x, p.y, p.z, 0, 0, 255);
			}
			for (octomap::ColorOcTree::leaf_iterator it = octo_model->begin_leafs(), end = octo_model->end_leafs(); it != end; ++it) {
				if (it->getOccupancy() > 0.65) {
					if (it.getZ() >= share_data->min_z_table + share_data->octomap_resolution) octo_model->integrateNodeColor(it.getKey(), 255, 0, 0);
					else octo_model->integrateNodeColor(it.getKey(), 0, 0, 255);
				}
			}
			octo_model->updateInnerOccupancy();

			share_data->clouds[view_space->views.size()]->~PointCloud();
			share_data->clouds.pop_back();

			share_data->access_directory(share_data->save_path);
			ofstream fout_grid(share_data->save_path + "/grid_toward" + to_string(toward_state) + "_rotate" + to_string(rotate_state) + "_view" + to_string(i) + ".txt");
			ofstream fout_view_ids(share_data->save_path + "/ids_toward" + to_string(toward_state) + "_rotate" + to_string(rotate_state) + "_view" + to_string(i) + ".txt");
			//octo_model->write(share_data->save_path + "/grid_toward" + to_string(toward_state) + "_rotate" + to_string(rotate_state) + "_view" + to_string(i) + ".ot");
			//octo_model->write(share_data->save_path + "/grid.ot");
			int num_of_squared_voxels = 0;
			//octomap::ColorOcTree* octo_model_square = new octomap::ColorOcTree(share_data->octomap_resolution);
			for (double x = share_data->object_center_world(0) - share_data->predicted_size; x <= share_data->object_center_world(0) + share_data->predicted_size; x += share_data->octomap_resolution)
				for (double y = share_data->object_center_world(1) - share_data->predicted_size; y <= share_data->object_center_world(1) + share_data->predicted_size; y += share_data->octomap_resolution)
					for (double z = share_data->object_center_world(2) - share_data->predicted_size; z <= share_data->object_center_world(2) + share_data->predicted_size; z += share_data->octomap_resolution)
					{
						auto node = octo_model->search(x, y, z);
						if (node == NULL) cout << "what?" << endl;
						//fout_grid << x - share_data->object_center_world(0) << ' ' << y - share_data->object_center_world(1) << ' ' << z - share_data->object_center_world(2) << ' ' << node->getOccupancy() << '\n';
						fout_grid << node->getOccupancy() << '\n';
						num_of_squared_voxels++;
						//octo_model_square->setNodeValue(x, y, z, node->getLogOdds(), true);
						//if (node->getOccupancy() > 0.65) {
						//	if(z >= share_data->min_z_table + share_data->octomap_resolution) octo_model_square->integrateNodeColor(x, y, z, 255, 0, 0);
						//	else octo_model_square->integrateNodeColor(x, y, z, 0, 0, 255);
						//}
					}
			if (num_of_squared_voxels != 32*32*32) cout << "voxels size wrong." << endl;
			for (int i = 0; i < need_views.size(); i++) {
				fout_view_ids << need_views[i] << '\n';
			}
			//octo_model_square->updateInnerOccupancy();
			//octo_model_square->write(share_data->save_path + "/square_grid.ot");
			//delete octo_model_square;
			delete octo_model;

			if (share_data->show) { //显示BBX、相机位置、GT
			//if (true) { //显示BBX、相机位置、GT
				pcl::visualization::PCLVisualizer::Ptr viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("Iteration"));
				viewer->setBackgroundColor(255, 255, 255);
				//viewer->addCoordinateSystem(0.05);
				viewer->initCameraParameters();
				//view_space->add_bbx_to_cloud(viewer);
				//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> gray(share_data->cloud_ground_truth, 128, 128, 128);
				//viewer->addPointCloud<pcl::PointXYZRGB>(share_data->cloud_ground_truth, gray, "cloud_ground_truth");
				//已选取位置
				view_space->views[i].get_next_camera_pos(Eigen::Matrix4d::Identity(4, 4), share_data->object_center_world);
				Eigen::Matrix4d view_pose_world = (Eigen::Matrix4d::Identity(4, 4) * view_space->views[i].pose.inverse()).eval();
				pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red(share_data->clouds[i], 255, 0, 0);
				viewer->addPointCloud<pcl::PointXYZRGB>(share_data->clouds[i], red, "cloud_i");
				//相机位置
				Eigen::Vector4d X(0.05, 0, 0, 1);
				Eigen::Vector4d Y(0, 0.05, 0, 1);
				Eigen::Vector4d Z(0, 0, 0.05, 1);
				Eigen::Vector4d O(0, 0, 0, 1);
				X = view_pose_world * X;
				Y = view_pose_world * Y;
				Z = view_pose_world * Z;
				O = view_pose_world * O;
				viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)), pcl::PointXYZ(X(0), X(1), X(2)), 255, 0, 0, "X-1");
				viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)), pcl::PointXYZ(Y(0), Y(1), Y(2)), 255, 0, 0, "Y-1");
				viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)), pcl::PointXYZ(Z(0), Z(1), Z(2)), 255, 0, 0, "Z-1");
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "X-1");
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "Y-1");
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "Z-1");
				for (int j = 0; j < need_views.size(); j++) {
					view_space->views[need_views[j]].get_next_camera_pos(Eigen::Matrix4d::Identity(4, 4), share_data->object_center_world);
					Eigen::Matrix4d view_pose_world = (Eigen::Matrix4d::Identity(4, 4) * view_space->views[need_views[j]].pose.inverse()).eval();
					//相机位置
					Eigen::Vector4d X(0.05, 0, 0, 1);
					Eigen::Vector4d Y(0, 0.05, 0, 1);
					Eigen::Vector4d Z(0, 0, 0.05, 1);
					Eigen::Vector4d O(0, 0, 0, 1);
					X = view_pose_world * X;
					Y = view_pose_world * Y;
					Z = view_pose_world * Z;
					O = view_pose_world * O;
					viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)), pcl::PointXYZ(X(0), X(1), X(2)), 255, 0, 0, "X" + to_string(j));
					viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)), pcl::PointXYZ(Y(0), Y(1), Y(2)), 0, 255, 0, "Y" + to_string(j));
					viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)), pcl::PointXYZ(Z(0), Z(1), Z(2)), 0, 0, 255, "Z" + to_string(j));
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "X" + to_string(j));
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "Y" + to_string(j));
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "Z" + to_string(j));
				}
				while (!viewer->wasStopped())
				{
					viewer->spinOnce(100);
					boost::this_thread::sleep(boost::posix_time::microseconds(100000));
				}
			}
			cout << "labed "<< i <<" getted with executed time " << clock() - now_time << " ms." << endl;
		}
		return 0;
	}

	~SC_NBV_Labeler() {
		delete view_space;
		delete percept;
	}
};

atomic<bool> stop = false;		//控制程序结束
Share_Data* share_data;			//共享数据区指针
SC_NBV_Labeler* labeler;

#define DebugOne 0
#define TestAll 1
#define TrainAll 2

int mode = DebugOne;

int main()
{
	//Init
	ios::sync_with_stdio(false);
	//选取模式
	if (mode == DebugOne)
	{
		//int part_num = 6;
		//cout << part_num << " thread, input index form 0:";
		//int index;
		//cin >> index;
		//NBV规划期初始化
		for (int i = 0; i < 6; i++)
		{
			//int i = index;
			for (int j = 0; j < 8; j++)
			{
				//数据区初始化
				//j = index;
				share_data = new Share_Data("../DefaultConfiguration.yaml", "", "");
				labeler = new SC_NBV_Labeler(share_data, i, j);
				if (share_data->init_voxels < 300) continue;
				labeler->label();
				delete labeler;
				delete share_data;
			}
		}
	}
	else if (mode == TestAll){
		//测试集
		vector<string> names;
		names.push_back("Armadillo");
		names.push_back("Dragon");
		names.push_back("Stanford_Bunny");
		names.push_back("Happy_Buddha");
		names.push_back("Thai_Statue");
		names.push_back("Lucy");
		names.push_back("LM1");
		names.push_back("LM2");
		names.push_back("LM3");
		names.push_back("LM4");
		names.push_back("LM5");
		names.push_back("LM6");
		names.push_back("LM7");
		names.push_back("LM8");
		names.push_back("LM9");
		names.push_back("LM10");
		names.push_back("LM11");
		names.push_back("LM12");
		names.push_back("obj_000001");
		names.push_back("obj_000002");
		names.push_back("obj_000003");
		names.push_back("obj_000004");
		names.push_back("obj_000005");
		names.push_back("obj_000006");
		names.push_back("obj_000007");
		names.push_back("obj_000008");
		names.push_back("obj_000009");
		names.push_back("obj_000010");
		names.push_back("obj_000011");
		names.push_back("obj_000012");
		names.push_back("obj_000013");
		names.push_back("obj_000014");
		names.push_back("obj_000015");
		names.push_back("obj_000016");
		names.push_back("obj_000017");
		names.push_back("obj_000018");
		names.push_back("obj_000019");
		names.push_back("obj_000020");
		names.push_back("obj_000021");
		names.push_back("obj_000022");
		names.push_back("obj_000023");
		names.push_back("obj_000024");
		names.push_back("obj_000025");
		names.push_back("obj_000026");
		names.push_back("obj_000027");
		names.push_back("obj_000028");
		names.push_back("obj_000029");
		names.push_back("obj_000030");
		names.push_back("obj_000031");
		//测试
		//int part_num = 4;
		//cout << part_num << " thread, input index form 0:";
		//int index;
		//cin >> index;
		for (int i = 0; i < names.size(); i++) {
		//for (int i = names.size() / part_num * index; i < names.size() / part_num * (index + 1); i++) {
			for (int j = 0; j < 6; j++){
				for (int k = 0; k < 8; k++) {
					//数据区初始化
					share_data = new Share_Data("../DefaultConfiguration.yaml", names[i], "");
					//NBV规划期初始化
					labeler = new SC_NBV_Labeler(share_data, j, k);
					if (share_data->init_voxels < 300) continue;
					labeler->label();
					delete labeler;
					delete share_data;
				}
			}
		}
	}
	else if (mode == TrainAll) {
		//测试集
		double now_time = clock();
		/*
		//remove_wrong
		vector<string> allPath;
		ifstream f_repath_in("../reneed_path.txt");
		string retemp_path;
		while (f_repath_in >> retemp_path) allPath.push_back(retemp_path);
		for (int i = 0; i < allPath.size(); i++)
		{
			string path = allPath[i];
			for (int i = 0; i < path.size(); i++) {
				if (path[i] == '\\') path[i] = '/';
			}
			string save_path = "../SC_label_data/" + path;
			cout << save_path << endl;
			remove((save_path + "/grid.txt").c_str());
			remove((save_path + "/view_ids.txt").c_str());
			cout << _rmdir(save_path.c_str()) << endl;
		}
		*/
		
		/*
		//check ans
		string shape_net_path = "D:\\Software\\PC-NBV\\models\\ShapeNetCore.v1";
		vector<string> allPath = getFilesList(shape_net_path);
		vector<string> reneedPath;
		for (int i = 0; i < allPath.size(); i++)
		{
			string path = allPath[i];
			for (int i = 0; i < path.size(); i++) {
				if (path[i] == '\\') path[i] = '/';
			}
			string save_path = "../SC_label_data/" + path;
			//cout << save_path << endl;
			ifstream f_grid(save_path + "/grid.txt");
			ifstream f_view(save_path + "/view_ids.txt");
			if (!f_grid.is_open() || !f_view.is_open()) {
				continue;
			}
			string line;
			int line_num = 0;
			while (getline(f_grid, line)) line_num++;
			int temp_index;
			int cnt_view = 0;
			while (f_view >> temp_index) cnt_view++;
			//cout << allPath[i] << endl;
			//cout << line_num << " , " << cnt_view << endl;
			if (line_num != 64000 || cnt_view <= 0) reneedPath.push_back(allPath[i]);
			if (i % 100 == 99) cout << i + 1 << " models checked. " << reneedPath.size() << " needed found." << endl;
		}
		ofstream f_repath_out("../reneed_path.txt");
		for (int i = 0; i < reneedPath.size(); i++) {
			f_repath_out << reneedPath[i] << endl;
		}
		*/
		/*
		//multi_reprocess测试
		vector<string> reneedPath;
		ifstream f_repath_in("../reneed_path.txt");
		string retemp_path;
		while (f_repath_in >> retemp_path) reneedPath.push_back(retemp_path);
		int repart_num = 8;
		cout << repart_num << " thread, input index form 0:";
		int reindex;
		cin >> reindex;
		for (int i = reneedPath.size() / repart_num * reindex; i < reneedPath.size() / repart_num * (reindex + 1); i++){
			share_data = new Share_Data("../DefaultConfiguration.yaml", "", reneedPath[i]);
			thread runner(get_run);
			runner.join();
			delete share_data;
		}
		return 1;
		*/

		//find path
		string shape_net_path = "D:\\Software\\PC-NBV\\models\\ShapeNetCore.v1";
		vector<string> allPath = getFilesList(shape_net_path);
		cout << allPath.size() << " file path readed with " << clock() - now_time << "ms." << endl;
		/*
		//get statics
		vector<int> num_of_case,sum_num_of_case;
		num_of_case.resize(64);
		sum_num_of_case.resize(64);
		for (int i = 0; i < 64; i++) {
			num_of_case[i] = sum_num_of_case[i] = 0;
		}
		for (int i = 0; i < allPath.size(); i++) {
			string path = allPath[i];
			for (int i = 0; i < path.size(); i++) {
				if (path[i] == '\\') path[i] = '/';
			}
			string save_path = "../SC_label_data/" + path;
			ifstream f_view(save_path + "/view_ids.txt");
			if (f_view.is_open()) {
				int cnt_view = 0;
				int temp_index;
				while (f_view >> temp_index) cnt_view++;
				num_of_case[cnt_view]++;
				//cout << save_path << "  : " << cnt_view << endl;
			}
		}
		sum_num_of_case[0] = num_of_case[0];
		cout<< 0 << '\t' << num_of_case[0] << '\t' << sum_num_of_case[0] << endl;
		for (int i = 1; i < 64; i++) {
			sum_num_of_case[i] = sum_num_of_case[i-1] + num_of_case[i];
			//cout << "view num " << i << ": " << num_of_case[i] << " ,sum: " << sum_num_of_case[i] << endl;
			cout << i << '\t' << num_of_case[i] << '\t' << sum_num_of_case[i] << endl;
		}
		return 1;
		*/
		//check path
		now_time = clock();
		vector<string> needPath;
		for (int i = 0; i < allPath.size(); i++) 
		{
			string path = allPath[i];
			for (int i = 0; i < path.size(); i++) {
				if (path[i] == '\\') path[i] = '/';
			}
			string save_path = "../SC_label_data/" + path;
			//cout << save_path << endl;
			ifstream f_grid(save_path +"/grid.txt");
			ifstream f_view(save_path +"/view_ids.txt");
			if (!f_grid.is_open() || !f_view.is_open()) {
				share_data = new Share_Data("../DefaultConfiguration.yaml", "", allPath[i]);
				labeler = new SC_NBV_Labeler(share_data);
				if (share_data->init_voxels >= 300)	needPath.push_back(allPath[i]);
				delete labeler;
				delete share_data;
			}
		}
		//out path
		ofstream f_path_out("../need_path.txt");
		for (int i = 0; i < needPath.size(); i++) {
			f_path_out << needPath[i] << endl;
		}
		
		/*
		//read path
		vector<string> needPath;
		ifstream f_path_in("../need_path.txt");
		string temp_path;
		while (f_path_in >> temp_path) needPath.push_back(temp_path);
		*/

		cout << needPath.size() << " needed file path getted with " << clock() - now_time << "ms." << endl;

		//multi_process测试
		int part_num = 8;
		cout << part_num << " thread, input index form 0:";
		int index;
		cin >> index;
		
		//for (int i = 0; i < needPath.size(); i++) {
		for (int i = needPath.size() / part_num * index; i < needPath.size() / part_num * (index + 1); i++){
			//NBV规划期初始化
			for (int i = 0; i < 6; i++) {
				for (int j = 0; j < 8; j++) {
					//数据区初始化
					share_data = new Share_Data("../DefaultConfiguration.yaml", "", needPath[i]);
					labeler = new SC_NBV_Labeler(share_data, i, j);
					if (share_data->init_voxels < 300) continue;
					labeler->label();
					delete labeler;
					delete share_data;
				}
			}
		}
	}
	cout << "System over." << endl;
	return 0;
}
