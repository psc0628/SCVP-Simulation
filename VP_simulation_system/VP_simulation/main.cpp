#include <windows.h>
#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
typedef unsigned long long pop_t;

using namespace std;

#include "Share_Data.hpp"
#include "View_Space.hpp"
#include "Information.hpp"

//Virtual_Perception_3D.hpp
void precept_thread_process(int i, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, octomap::point3d* _origin, octomap::point3d* _end, Eigen::Matrix4d* _view_pose_world, octomap::ColorOcTree* _ground_truth_model, Share_Data* share_data);

class Perception_3D {
public:
	Share_Data* share_data;
	octomap::ColorOcTree* ground_truth_model;
	int full_voxels;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

	Perception_3D(Share_Data* _share_data) {
		share_data = _share_data;
		ground_truth_model = new octomap::ColorOcTree(share_data->ground_truth_resolution);
		for (octomap::ColorOcTree::leaf_iterator it = share_data->ground_truth_model->begin_leafs(), end = share_data->ground_truth_model->end_leafs(); it != end; ++it) {
			ground_truth_model->setNodeValue(it.getX(), it.getY(), it.getZ(), it->getLogOdds(), true);
		}
		for (double x = share_data->object_center_world(0) - 0.2; x <= share_data->object_center_world(0) + 0.2; x += share_data->ground_truth_resolution)
			for (double y = share_data->object_center_world(2) - 0.2; y <= share_data->object_center_world(2) + 0.2; y += share_data->ground_truth_resolution) {
				double z = share_data->min_z_table;
				ground_truth_model->setNodeValue(x, y, z, ground_truth_model->getProbHitLog(), true);
			}
		full_voxels = 0;
		for (octomap::ColorOcTree::leaf_iterator it = ground_truth_model->begin_leafs(), end = ground_truth_model->end_leafs(); it != end; ++it) {
			full_voxels++;
		}
	}

	~Perception_3D() {
		;
	}

	bool precept(View* now_best_view) {
		double now_time = clock();
		//������ǰ�������
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_parallel(new pcl::PointCloud<pcl::PointXYZRGB>);
		cloud_parallel->is_dense = false;
		cloud_parallel->points.resize(full_voxels);
		//��ȡ�ӵ�λ��
		Eigen::Matrix4d view_pose_world;
		now_best_view->get_next_camera_pos(share_data->now_camera_pose_world, share_data->object_center_world);
		view_pose_world = (share_data->now_camera_pose_world * now_best_view->pose.inverse()).eval();
		//����ӵ��key
		octomap::OcTreeKey key_origin;
		bool key_origin_have = ground_truth_model->coordToKeyChecked(now_best_view->init_pos(0), now_best_view->init_pos(1), now_best_view->init_pos(2), key_origin);
		if (key_origin_have) {
			octomap::point3d origin = ground_truth_model->keyToCoord(key_origin);
			//����ÿ������
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
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr no_table(new pcl::PointCloud<pcl::PointXYZRGB>);
		cloud = temp;
		cloud->is_dense = false;
		no_table->is_dense = false;
		cloud->points.resize(full_voxels);
		no_table->points.resize(full_voxels);
		auto ptr = cloud->points.begin();
		auto pt = no_table->points.begin();
		int vaild_point = 0;
		int table_point = 0;
		auto p = cloud_parallel->points.begin();
		for (int i = 0; i < cloud_parallel->points.size(); i++, p++)
		{
			if ((*p).x == 0 && (*p).y == 0 && (*p).z == 0) continue;
			if ((*p).z > share_data->min_z_table + share_data->ground_truth_resolution) {
				(*pt).x = (*p).x;
				(*pt).y = (*p).y;
				(*pt).z = (*p).z;
				(*pt).b = (*p).b;
				(*pt).g = (*p).g;
				(*pt).r = (*p).r;
				table_point++;
				pt++;
			}
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
		no_table->width = table_point;
		cloud->height = 1;
		no_table->height = 1;
		cloud->points.resize(vaild_point);
		no_table->points.resize(table_point);
		//��¼��ǰ�ɼ�����
		share_data->vaild_clouds++;
		share_data->clouds.push_back(cloud);
		//��ת����������ϵ
		*share_data->cloud_final += *no_table;
		//cout << "virtual cloud get with executed time " << clock() - now_time << " ms." << endl;
		if (share_data->show) { //��ʾ�������
			pcl::visualization::PCLVisualizer::Ptr viewer1(new pcl::visualization::PCLVisualizer("Camera"));
			viewer1->setBackgroundColor(0, 0, 0);
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
			while (!viewer1->wasStopped())
			{
				viewer1->spinOnce(100);
				boost::this_thread::sleep(boost::posix_time::microseconds(100000));
			}
		}

		cloud_parallel->~PointCloud();
		no_table->~PointCloud();
		cout <<"Virtual cloud getted with time "<< clock() - now_time<<" ms." << endl;
		return true;
	}
};

void precept_thread_process(int i, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, octomap::point3d* _origin, octomap::point3d* _end, Eigen::Matrix4d* _view_pose_world, octomap::ColorOcTree* _ground_truth_model, Share_Data* share_data) {
	//num++;
	octomap::point3d origin = *_origin;
	Eigen::Matrix4d view_pose_world = *_view_pose_world;
	octomap::ColorOcTree* ground_truth_model = _ground_truth_model;
	pcl::PointXYZRGB point;
	point.x = 0; point.y = 0; point.z = 0;
	//ͶӰ����Ƿ��ڳ���Χ��
	Eigen::Vector4d end_3d(_end->x(), _end->y(), _end->z(), 1);
	Eigen::Vector4d vertex = view_pose_world.inverse() * end_3d;
	float point_3d[3] = { vertex(0), vertex(1),vertex(2) };
	float pixel[2];
	rs2_project_point_to_pixel(pixel, &share_data->color_intrinsics, point_3d);
	if (pixel[0] < 0 || pixel[0]>share_data->color_intrinsics.width || pixel[1] < 0 || pixel[1]>share_data->color_intrinsics.height) {
		cloud->points[i] = point;
		return;
	}
	//����ͶӰ�ҵ��յ�
	octomap::point3d end = project_pixel_to_ray_end(pixel[0], pixel[1], share_data->color_intrinsics, view_pose_world, 1.0);
	octomap::OcTreeKey key_end;
	octomap::point3d direction = end - origin;
	octomap::point3d end_point;
	//Խ��δ֪�����ҵ��յ�
	bool found_end_point = ground_truth_model->castRay(origin, direction, end_point, true, 6.0 * share_data->predicted_size);
	if (!found_end_point) {//δ�ҵ��յ㣬�޹۲�����
		cloud->points[i] = point;
		return;
	}
	if (end_point == origin) {
		cout << "view in the object. check!" << endl;
		cloud->points[i] = point;
		return;
	}
	//���һ��ĩ���Ƿ��ڵ�ͼ���Ʒ�Χ��
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

//Global_Path_Planner.hpp
class Global_Path_Planner {
public:
	Share_Data* share_data;
	View_Space* view_space;
	View* now_view;
	bool solved;
	long long n,m;
	map<int, int>* view_id_in;
	map<int, int>* view_id_out;
	vector<vector<double>> graph;
	vector<vector<double>> dp;
	vector<vector<int>> path;
	double total_shortest;
	vector<int> global_path;

	Global_Path_Planner(Share_Data* _share_data, View_Space* _view_space, View* _now_view, vector<int>& view_set_label) {
		share_data = _share_data;
		view_space = _view_space;
		now_view = _now_view;
		solved = false;
		//�����±�ӳ��
		view_id_in = new map<int, int>();
		view_id_out = new map<int, int>();
		(*view_id_in)[now_view->id] = 0;
		(*view_id_out)[0] = now_view->id;
		for (int i = 0; i < view_set_label.size(); i++) {
			(*view_id_in)[view_set_label[i]] = i+1;
			(*view_id_out)[i+1] = view_set_label[i];
		}
		//�ڵ����뷽����
		n = view_set_label.size() + 1;
		m = 1LL << n;
		//local path ��ȫ����ͼ
		graph.resize(n);
		for (int i = 0; i < n; i++)
			graph[i].resize(n);
		for (int i = 0; i < n; i++)
			for (int j = 0; j < n; j++) {
				//����id
				int u = (*view_id_out)[i];
				int v = (*view_id_out)[j];
				//������·��
				pair<int, double> local_path = get_local_path(view_space->views[u].init_pos.eval(), view_space->views[v].init_pos.eval(), view_space->object_center_world.eval(), view_space->predicted_size * sqrt(2)); //��Χ�а뾶�ǰ�߳��ĸ���2��
				if (local_path.first < 0) cout << "local path wrong." << endl;
				graph[i][j] = local_path.second;
			}
		//��ʼ��dp
		dp.resize(m);
		for (int i = 0; i < m; i++)
			dp[i].resize(n);
		for (int i = 0; i < m; i++)
			for (int j = 0; j < n; j++)
				dp[i][j] = 1e10;
		dp[1][0] = 0;
		//��ʼ��·����¼
		path.resize(m);
		for (int i = 0; i < m; i++)
			path[i].resize(n);
		cout << "Global_Path_Planner inited." << endl;
	}

	~Global_Path_Planner() {
		delete view_id_in;
		delete view_id_out;
		graph.clear();
		dp.clear();
		path.clear();
		global_path.clear();
	}

	double solve() {
		double now_time = clock();
		for (int i = 0; i < m; i++)  // i�������һ�������ļ��ϣ�����ÿ��λ�õ�0/1����û��/�о��������(m=1<<n)
		{
			for (int j = 0; j < n; j++)  //ö�ٵ�ǰ�����ĸ���
			{
				if ((i >> j) & 1)  //���i�����j
				{
					for (int k = 0; k < n; k++)  //ö�ٵ���j�ĵ�
					{
						if (i - (1 << j) >> k & 1)  //ȥ������j�ļ���i
						{
							//�����������²���¼ת��·��
							if (dp[i][j] >= dp[i - (1LL << j)][k] + graph[k][j]) {
								dp[i][j] = dp[i - (1LL << j)][k] + graph[k][j];
								path[i][j] = k;
							}
							// dp[i][j] = min(dp[i][j], dp[i - (1 << j)][k] + mp[k][j]);
						}
					}
				}
			}
		}
		//Ĭ�����Ϊ0,����ÿ�������յ��ҵ���̹��ܶ���·����ȫ���Ƿ�����
		int end_node;
		total_shortest = 1e20;
		for (int i = 1; i < n; i++) {
			if (total_shortest > dp[m - 1][i]) {
				total_shortest = dp[m - 1][i];
				end_node = i;
			}
		}
		//���·��
		for (int i = (1 << n) - 1, j = end_node; i > 0;) {
			//ע���±꽻��
			global_path.push_back((*view_id_out)[j]);
			int ii = i - (1 << j);
			int jj = path[i][j];
			i = ii, j = jj;
		}
		//����·���跴��
		reverse(global_path.begin(), global_path.end());
		solved = true;
		double cost_time = clock() - now_time;
		cout << "Global Path length " << total_shortest << " getted with executed time " << cost_time << " ms." << endl;
		//����
		share_data->access_directory(share_data->save_path + "/movement");
		ofstream fout_global_path(share_data->save_path + "/movement/global_path.txt");
		fout_global_path << total_shortest << '\t' << cost_time << '\t' << endl;
		return total_shortest;
	}

	vector<int> get_path_id_set() {
		if (!solved) cout << "call solve() first" << endl;
		cout << "Node ids on global_path form start to end are: ";
		for (int i = 0; i < global_path.size(); i++)
			cout << global_path[i] << " ";
		cout << endl;
		//ɾ��������
		vector<int> ans;
		ans = global_path;
		ans.erase(ans.begin());
		return ans;
	}
};

#define Over 0
#define WaitData 1
#define WaitViewSpace 2
#define WaitInformation 3
#define WaitMoving 4

//NVB_Planner.hpp
void save_cloud_mid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, string name);
void create_view_space(View_Space** now_view_space, View* now_best_view, Share_Data* share_data, int iterations);
void create_views_information(Views_Information** now_views_infromation, View* now_best_view, View_Space* now_view_space, Share_Data* share_data, int iterations);
void move_robot(View* now_best_view, View_Space* now_view_space, Share_Data* share_data);
void show_cloud(pcl::visualization::PCLVisualizer::Ptr viewer);

class NBV_Planner
{
public:
	atomic<int> status;
	int iterations;
	Perception_3D* percept;
	Voxel_Information* voxel_information;
	View_Space* now_view_space;
	Views_Information* now_views_infromation;
	View* now_best_view;
	Share_Data* share_data;
	pcl::visualization::PCLVisualizer::Ptr viewer;

	~NBV_Planner() {
		delete percept;
		delete now_best_view;
		delete voxel_information;
		delete now_view_space;
		//ֻ������������information
		if (share_data->method_of_IG == 6 || share_data->method_of_IG == 7);
		else delete now_views_infromation;
	}

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

	NBV_Planner(Share_Data* _share_data, int _status = WaitData) {
		share_data = _share_data;
		iterations = 0;
		status = _status;
		share_data->now_view_space_processed = false;
		share_data->now_views_infromation_processed = false;
		share_data->move_on = false;
		voxel_information = new Voxel_Information(share_data->p_unknown_lower_bound, share_data->p_unknown_upper_bound);
		voxel_information->init_mutex_views(share_data->num_of_views);
		//��ʼ��GT
		share_data->access_directory(share_data->save_path);
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
		//��������С��ͳһ����Ϊ0.15m����
		vector<Eigen::Vector3d> points;
		for (auto& ptr : share_data->cloud_pcd->points) {
			Eigen::Vector3d pt(ptr.x * unit, ptr.y * unit, ptr.z * unit);
			points.push_back(pt);
		}
		Eigen::Vector3d object_center_world = Eigen::Vector3d(0, 0, 0);
		//�����������
		for (auto& ptr : points) {
			object_center_world(0) += ptr(0);
			object_center_world(1) += ptr(1);
			object_center_world(2) += ptr(2);
		}
		object_center_world(0) /= points.size();
		object_center_world(1) /= points.size();
		object_center_world(2) /= points.size();
		//���ֲ���BBX�뾶����BBX�ڵ�ĸ������ʴﵽ0.90-0.95Ϊ��ֹ����
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
		//ת������
		double min_z = object_center_world(2);
		for (int i = 0; i < share_data->cloud_pcd->points.size(); i++, p++)
		{
			(*ptr).x = (*p).x * scale * unit;
			(*ptr).y = (*p).y * scale * unit;
			(*ptr).z = (*p).z * scale * unit;
			(*ptr).b = 168;
			(*ptr).g = 168;
			(*ptr).r = 168;
			//GT�������
			octomap::OcTreeKey key;  bool key_have = share_data->ground_truth_model->coordToKeyChecked(octomap::point3d((*ptr).x, (*ptr).y, (*ptr).z), key);
			if (key_have) {
				octomap::ColorOcTreeNode* voxel = share_data->ground_truth_model->search(key);
				if (voxel == NULL) {
					share_data->ground_truth_model->setNodeValue(key, share_data->ground_truth_model->getProbHitLog(), true);
					share_data->ground_truth_model->integrateNodeColor(key, (*ptr).r, (*ptr).g, (*ptr).b);
				}
			}
			min_z = min(min_z, (double)(*ptr).z);
			//GT_sample�������
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
		//��¼����
		share_data->min_z_table = min_z - share_data->ground_truth_resolution;

		share_data->ground_truth_model->updateInnerOccupancy();
		share_data->ground_truth_model->write(share_data->save_path + "/GT.ot");
		//GT_sample_voxels
		share_data->GT_sample->updateInnerOccupancy();
		share_data->GT_sample->write(share_data->save_path + "/GT_sample.ot");
		share_data->init_voxels = 0;
		int full_voxels = 0;
		//��sample��ͳ���ܸ���
		for (octomap::ColorOcTree::leaf_iterator it = share_data->GT_sample->begin_leafs(), end = share_data->GT_sample->end_leafs(); it != end; ++it){
			if(it.getY() > share_data->min_z_table + share_data->octomap_resolution)
				share_data->init_voxels++;
			full_voxels++;
		}
		/*//��GT��ͳ���ܸ���
		for (octomap::ColorOcTree::leaf_iterator it = share_data->ground_truth_model->begin_leafs(), end = share_data->ground_truth_model->end_leafs(); it != end; ++it) {
			if (it.getY() > share_data->min_z_table + share_data->octomap_resolution)
				share_data->init_voxels++;
			full_voxels++;
		}*/
		cout << "Map_GT_sample has voxels " << share_data->init_voxels << endl;
		cout << "Map_GT_sample has voxels with bottom " << full_voxels << endl;
		share_data->init_voxels = full_voxels;
		ofstream fout(share_data->save_path + "/GT_sample_voxels.txt");
		fout << share_data->init_voxels << endl;

		//��ʼ��viewspace
		int first_view_id = 0;
		now_view_space = new View_Space(iterations, share_data, voxel_information, share_data->cloud_ground_truth, first_view_id);
		//���ó�ʼ�ӵ�Ϊͳһ��λ��
		//if (share_data->method_of_IG == 7) { //SC-NET
			now_view_space->views[first_view_id].vis++;
			now_best_view = new View(now_view_space->views[first_view_id]);
		//}
		/*else {
			now_view_space->views[0].vis++;
			now_best_view = new View(now_view_space->views[0]);
		}*/
		//�˶����ۣ��ӵ�id����ǰ���ۣ��������
		share_data->movement_cost = 0;
		share_data->access_directory(share_data->save_path + "/movement");
		ofstream fout_move(share_data->save_path + "/movement/path" + to_string(-1) + ".txt");
		fout_move << 0 << '\t' << 0.0 << '\t' << 0.0 << endl;
		now_best_view->get_next_camera_pos(share_data->now_camera_pose_world, share_data->object_center_world);
		Eigen::Matrix4d view_pose_world = (share_data->now_camera_pose_world * now_best_view->pose.inverse()).eval();
		//������ʼ��
		percept = new Perception_3D(share_data);
		if (share_data->show) { //��ʾBBX�����λ�á�GT
			pcl::visualization::PCLVisualizer::Ptr viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("Iteration"));
			viewer->setBackgroundColor(0, 0, 0);
			viewer->addCoordinateSystem(0.1);
			viewer->initCameraParameters();
			//��һ֡���λ��
			Eigen::Vector4d X(0.05, 0, 0, 1);
			Eigen::Vector4d Y(0, 0.05, 0, 1);
			Eigen::Vector4d Z(0, 0, 0.05, 1);
			Eigen::Vector4d O(0, 0, 0, 1);
			X = view_pose_world * X;
			Y = view_pose_world * Y;
			Z = view_pose_world * Z;
			O = view_pose_world * O;
			viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)), pcl::PointXYZ(X(0), X(1), X(2)), 255, 0, 0, "X" + to_string(-1));
			viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)), pcl::PointXYZ(Y(0), Y(1), Y(2)), 0, 255, 0, "Y" + to_string(-1));
			viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)), pcl::PointXYZ(Z(0), Z(1), Z(2)), 0, 0, 255, "Z" + to_string(-1));
			//test_viewspace
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr test_viewspace(new pcl::PointCloud<pcl::PointXYZRGB>);
			test_viewspace->is_dense = false;
			test_viewspace->points.resize(now_view_space->views.size());
			auto pt = test_viewspace->points.begin();
			for (int i = 0; i < now_view_space->views.size(); i++, pt++) {
				(*pt).x = now_view_space->views[i].init_pos(0);
				(*pt).y = now_view_space->views[i].init_pos(1);
				(*pt).z = now_view_space->views[i].init_pos(2);
				//��һ����ʾ���е�Ϊ��ɫ
				(*pt).r = 255, (*pt).g = 255, (*pt).b = 255;
			}
			viewer->addPointCloud<pcl::PointXYZRGB>(test_viewspace, "test_viewspace");
			now_view_space->add_bbx_to_cloud(viewer);
			viewer->addPointCloud<pcl::PointXYZRGB>(share_data->cloud_ground_truth, "cloud_ground_truth");
			while (!viewer->wasStopped())
			{
				viewer->spinOnce(100);
				boost::this_thread::sleep(boost::posix_time::microseconds(100000));
			}
		}
	}

	int plan() {
		switch (status)
		{
		case Over:
			break;
		case WaitData:
			if (percept->precept(now_best_view)) {
				thread next_view_space(create_view_space, &now_view_space, now_best_view, share_data, iterations);
				next_view_space.detach();
				status = WaitViewSpace;
			}
			break;
		case WaitViewSpace:
			if (share_data->now_view_space_processed) {
				thread next_views_information(create_views_information, &now_views_infromation, now_best_view, now_view_space, share_data, iterations);
				next_views_information.detach();
				status = WaitInformation;
			}
			break;
		case WaitInformation:
			if (share_data->now_views_infromation_processed) {
				if (share_data->method_of_IG == 6) { //NBV-NET
					share_data->access_directory(share_data->nbv_net_path + "/log");
					ifstream ftest;
					do {
						ftest.open(share_data->nbv_net_path + "/log/ready.txt");
					} while (!ftest.is_open());
					ftest.close();
					ifstream fin(share_data->nbv_net_path + "/log/" + share_data->name_of_pcd + '_' + to_string(iterations) + ".txt");
					int id;
					fin >> id;
					cout <<"next view id is "<< id << endl;
					delete now_best_view;
					now_best_view = new View(now_view_space->views[id]);
					this_thread::sleep_for(chrono::seconds(5));
					int removed = remove((share_data->nbv_net_path + "/log/ready.txt").c_str());
					if (removed!=0) cout << "cannot remove ready.txt." << endl;
					//�˶����ۣ��ӵ�id����ǰ���ۣ��������
					share_data->movement_cost += now_best_view->robot_cost;
					share_data->access_directory(share_data->save_path + "/movement");
					ofstream fout_move(share_data->save_path + "/movement/path" + to_string(iterations) + ".txt");
					fout_move << now_best_view->id << '\t' << now_best_view->robot_cost << '\t' << share_data->movement_cost << endl;
				}
				else if (share_data->method_of_IG == 7) { //SCVP
					if (iterations == 0) {
						share_data->access_directory(share_data->sc_net_path + "/log");
						ifstream ftest;
						do {
							ftest.open(share_data->sc_net_path + "/log/ready.txt");
						} while (!ftest.is_open());
						ftest.close();
						ifstream fin(share_data->sc_net_path + "/log/" + share_data->name_of_pcd + ".txt");
						vector<int> view_set_label;
						int rest_view_id;
						while (fin >> rest_view_id) {
							view_set_label.push_back(rest_view_id);
						}
						//�滮·��
						Global_Path_Planner* gloabl_path_planner = new Global_Path_Planner(share_data, now_view_space, now_best_view, view_set_label);
						gloabl_path_planner->solve();
						//����·��
						share_data->view_label_id = gloabl_path_planner->get_path_id_set();
						delete now_best_view;
						now_best_view = new View(now_view_space->views[share_data->view_label_id[iterations]]);
						//�˶����ۣ��ӵ�id����ǰ���ۣ��������
						share_data->movement_cost += now_best_view->robot_cost;
						share_data->access_directory(share_data->save_path + "/movement");
						ofstream fout_move(share_data->save_path + "/movement/path" + to_string(iterations) + ".txt");
						fout_move << now_best_view->id << '\t' << now_best_view->robot_cost << '\t' << share_data->movement_cost << endl;
						//���±�־�ļ�
						this_thread::sleep_for(chrono::seconds(5));
						int removed = remove((share_data->sc_net_path + "/log/ready.txt").c_str());
						if (removed != 0) cout << "cannot remove ready.txt." << endl;
					}
					else {
						if (iterations == share_data->view_label_id.size()) {
							share_data->over = true;
							status = WaitMoving;
							break;
						}
						delete now_best_view;
						now_best_view = new View(now_view_space->views[share_data->view_label_id[iterations]]);
						//�˶����ۣ��ӵ�id����ǰ���ۣ��������
						share_data->movement_cost += now_best_view->robot_cost;
						share_data->access_directory(share_data->save_path + "/movement");
						ofstream fout_move(share_data->save_path + "/movement/path" + to_string(iterations) + ".txt");
						fout_move << now_best_view->id << '\t' << now_best_view->robot_cost << '\t' << share_data->movement_cost << endl;
					}
				}
				else{//�����㷨
					//���ӵ�����
					sort(now_view_space->views.begin(), now_view_space->views.end(), view_utility_compare);
					/*if (share_data->sum_local_information == 0) {
						cout << "randomly choose a view" << endl;
						srand(clock());
						random_shuffle(now_view_space->views.begin(), now_view_space->views.end());
					}*/
					//informed_viewspace
					if (share_data->show) { //��ʾBBX�����λ��
						pcl::visualization::PCLVisualizer::Ptr viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("Iteration" + to_string(iterations)));
						viewer->setBackgroundColor(0, 0, 0);
						viewer->addCoordinateSystem(0.1);
						viewer->initCameraParameters();
						//test_viewspace
						pcl::PointCloud<pcl::PointXYZRGB>::Ptr test_viewspace(new pcl::PointCloud<pcl::PointXYZRGB>);
						test_viewspace->is_dense = false;
						test_viewspace->points.resize(now_view_space->views.size());
						auto ptr = test_viewspace->points.begin();
						int needed = 0;
						for (int i = 0; i < now_view_space->views.size(); i++) {
							(*ptr).x = now_view_space->views[i].init_pos(0);
							(*ptr).y = now_view_space->views[i].init_pos(1);
							(*ptr).z = now_view_space->views[i].init_pos(2);
							//���ʹ��ĵ��¼Ϊ��ɫ
							if (now_view_space->views[i].vis) (*ptr).r = 0, (*ptr).g = 0, (*ptr).b = 255;
							//���������ڵ�����Ϊ��ɫ
							else if (now_view_space->views[i].in_coverage[iterations] && i < now_view_space->views.size() / 10) (*ptr).r = 255, (*ptr).g = 255, (*ptr).b = 0;
							//���������ڵ�����Ϊ��ɫ
							else if (now_view_space->views[i].in_coverage[iterations]) (*ptr).r = 255, (*ptr).g = 0, (*ptr).b = 0;
							//ǰ10%��Ȩ�صĵ�����Ϊ����ɫ
							else if (i < now_view_space->views.size() / 10) (*ptr).r = 0, (*ptr).g = 255, (*ptr).b = 255;
							//����㲻Ҫ��
							else continue;
							ptr++;
							needed++;
						}
						test_viewspace->points.resize(needed);
						viewer->addPointCloud<pcl::PointXYZRGB>(test_viewspace, "test_viewspace");
						bool best_have = false;
						for (int i = 0; i < now_view_space->views.size(); i++) {
							if (now_view_space->views[i].vis) {
								now_view_space->views[i].get_next_camera_pos(share_data->now_camera_pose_world, share_data->object_center_world);
								Eigen::Matrix4d view_pose_world = (share_data->now_camera_pose_world * now_view_space->views[i].pose.inverse()).eval();
								Eigen::Vector4d X(0.03, 0, 0, 1);
								Eigen::Vector4d Y(0, 0.03, 0, 1);
								Eigen::Vector4d Z(0, 0, 0.03, 1);
								Eigen::Vector4d O(0, 0, 0, 1);
								X = view_pose_world * X;
								Y = view_pose_world * Y;
								Z = view_pose_world * Z;
								O = view_pose_world * O;
								viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)), pcl::PointXYZ(X(0), X(1), X(2)), 255, 0, 0, "X" + to_string(i));
								viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)), pcl::PointXYZ(Y(0), Y(1), Y(2)), 0, 255, 0, "Y" + to_string(i));
								viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)), pcl::PointXYZ(Z(0), Z(1), Z(2)), 0, 0, 255, "Z" + to_string(i));
							}
							else if (!best_have) {
								now_view_space->views[i].get_next_camera_pos(share_data->now_camera_pose_world, share_data->object_center_world);
								Eigen::Matrix4d view_pose_world = (share_data->now_camera_pose_world * now_view_space->views[i].pose.inverse()).eval();
								Eigen::Vector4d X(0.08, 0, 0, 1);
								Eigen::Vector4d Y(0, 0.08, 0, 1);
								Eigen::Vector4d Z(0, 0, 0.08, 1);
								Eigen::Vector4d O(0, 0, 0, 1);
								X = view_pose_world * X;
								Y = view_pose_world * Y;
								Z = view_pose_world * Z;
								O = view_pose_world * O;
								viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)), pcl::PointXYZ(X(0), X(1), X(2)), 255, 0, 0, "X" + to_string(i));
								viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)), pcl::PointXYZ(Y(0), Y(1), Y(2)), 0, 255, 0, "Y" + to_string(i));
								viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)), pcl::PointXYZ(Z(0), Z(1), Z(2)), 0, 0, 255, "Z" + to_string(i));
								best_have = true;
							}
						}
						viewer->addPointCloud<pcl::PointXYZRGB>(share_data->cloud_final, "cloud_now_itreation");
						while (!viewer->wasStopped())
						{
							viewer->spinOnce(100);
							boost::this_thread::sleep(boost::posix_time::microseconds(100000));
						}
					}
					double max_utility = -1;
					for (int i = 0; i < now_view_space->views.size(); i++) {
						cout << "checking view " << i << endl;
						if (now_view_space->views[i].vis) continue;
						//if (!now_view_space->views[i].can_move) continue;
						delete now_best_view;
						now_best_view = new View(now_view_space->views[i]);
						max_utility = now_best_view->final_utility;
						now_view_space->views[i].vis++;
						now_view_space->views[i].can_move = true;
						cout << "choose the " << i << "th view." << endl;
						//�˶����ۣ��ӵ�id����ǰ���ۣ��������
						share_data->movement_cost += now_best_view->robot_cost;
						share_data->access_directory(share_data->save_path + "/movement");
						ofstream fout_move(share_data->save_path + "/movement/path" + to_string(iterations) + ".txt");
						fout_move << now_best_view->id << '\t' << now_best_view->robot_cost << '\t' << share_data->movement_cost << endl;
						break;
					}
					if (max_utility == -1) {
						cout << "Can't move to any viewport.Stop." << endl;
						status = Over;
						break;
					}
					cout << " next best view pos is (" << now_best_view->init_pos(0) << ", " << now_best_view->init_pos(1) << ", " << now_best_view->init_pos(2) << ")" << endl;
					cout << " next best view final_utility is " << now_best_view->final_utility << endl;
				}
				//�����˶�ģ��
				thread next_moving(move_robot, now_best_view, now_view_space, share_data);
				next_moving.detach();
				status = WaitMoving;
			}
			break;
		case WaitMoving:
			//virtual move
			if (share_data->over) {
				cout << "Progress over.Saving octomap and cloud." << endl;
				status = Over;
				break;
			}
			if (share_data->move_on) {
				iterations++;
				share_data->now_view_space_processed = false;
				share_data->now_views_infromation_processed = false;
				share_data->move_on = false;
				status = WaitData;
			}
			break;
		}
		return status;
	}

	string out_status() {
		string status_string;
		switch (status)
		{
		case Over:
			status_string = "Over";
			break;
		case WaitData:
			status_string = "WaitData";
			break;
		case WaitViewSpace:
			status_string = "WaitViewSpace";
			break;
		case WaitInformation:
			status_string = "WaitInformation";
			break;
		case WaitMoving:
			status_string = "WaitMoving";
			break;
		}
		return status_string;
	}
};

atomic<bool> stop = false;		//���Ƴ������
Share_Data* share_data;			//����������ָ��
NBV_Planner* nbv_plan;

void save_cloud_mid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, string name) {
	//�����м�ĵ��Ƶ��̣߳�Ŀǰ������Ƿ񱣴����
	share_data->save_cloud_to_disk(cloud, "/clouds", name);
	cout << name << " saved" << endl;
}

void create_view_space(View_Space** now_view_space, View* now_best_view, Share_Data* share_data, int iterations) {
	//����ؼ�֡���λ��
	share_data->now_camera_pose_world = (share_data->now_camera_pose_world * now_best_view->pose.inverse()).eval();;
	//����viewspace
	(*now_view_space)->update(iterations, share_data, share_data->cloud_final, share_data->clouds[iterations]);
	//�����м�������
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_mid(new pcl::PointCloud<pcl::PointXYZRGB>);
	*cloud_mid = *share_data->cloud_final;
	thread save_mid(save_cloud_mid, cloud_mid, "pointcloud" + to_string(iterations));
	save_mid.detach();
	//���±�־λ
	share_data->now_view_space_processed = true;
}

void create_views_information(Views_Information** now_views_infromation, View* now_best_view, View_Space* now_view_space, Share_Data* share_data, int iterations) {
	if (share_data->method_of_IG == 6) { //NBV-NET
		//octotree
		share_data->access_directory(share_data->nbv_net_path + "/data");
		ofstream fout(share_data->nbv_net_path + "/data/" + share_data->name_of_pcd +'_'+ to_string(iterations) + ".txt");
		int num_of_square_voxels = 0;
		for (double x = now_view_space->object_center_world(0) - now_view_space->predicted_size; x <= now_view_space->object_center_world(0) + now_view_space->predicted_size; x += share_data->octomap_resolution)
			for (double y = now_view_space->object_center_world(1) - now_view_space->predicted_size; y <= now_view_space->object_center_world(1) + now_view_space->predicted_size; y += share_data->octomap_resolution)
				for (double z = now_view_space->object_center_world(2) - now_view_space->predicted_size; z <= now_view_space->object_center_world(2) + now_view_space->predicted_size; z += share_data->octomap_resolution)
				{
					auto node = share_data->octo_model->search(x, y, z);
					if (node == NULL) cout << "what?" << endl;
					fout << node->getOccupancy() << '\n';
					num_of_square_voxels++;
				}
		if (num_of_square_voxels != 32 * 32 * 32) cout << "what? no square." << endl;
	}
	else if (share_data->method_of_IG == 7) { //SCVP
		if (iterations == 0) {
			//octotree
			share_data->access_directory(share_data->sc_net_path + "/data");
			ofstream fout(share_data->sc_net_path + "/data/" + share_data->name_of_pcd + ".txt");
			int num_of_square_voxels = 0;
			for (double x = now_view_space->object_center_world(0) - now_view_space->predicted_size; x <= now_view_space->object_center_world(0) + now_view_space->predicted_size; x += share_data->octomap_resolution)
				for (double y = now_view_space->object_center_world(1) - now_view_space->predicted_size; y <= now_view_space->object_center_world(1) + now_view_space->predicted_size; y += share_data->octomap_resolution)
					for (double z = now_view_space->object_center_world(2) - now_view_space->predicted_size; z <= now_view_space->object_center_world(2) + now_view_space->predicted_size; z += share_data->octomap_resolution)
					{
						auto node = share_data->octo_model->search(x, y, z);
						if (node == NULL) cout << "what?" << endl;
						fout << node->getOccupancy() << '\n';
						num_of_square_voxels++;
					}
			if (num_of_square_voxels != 32 * 32 * 32) cout << "what? no square." << endl;
		}
	}
	else { //��������
		int num_of_cover = 0;
		//����views_informaiton
		if (iterations == 0) (*now_views_infromation) = new Views_Information(share_data, nbv_plan->voxel_information, now_view_space, iterations);
		else (*now_views_infromation)->update(share_data, now_view_space, iterations);
		if (share_data->method_of_IG == MCMF) {
			//��������������ȡȫ���Ż�����
			views_voxels_MF* set_cover_solver = new views_voxels_MF(share_data->num_of_max_flow_node, now_view_space, *now_views_infromation, nbv_plan->voxel_information, share_data);
			set_cover_solver->solve();
			vector<int> coverage_view_id_set = set_cover_solver->get_view_id_set();
			for (int i = 0; i < coverage_view_id_set.size(); i++)
				now_view_space->views[coverage_view_id_set[i]].in_coverage[iterations] = 1;
			delete set_cover_solver;
		}
		//�ۺϼ���ֲ�̰����ȫ���Ż��������ӵ���Ϣ��
		share_data->sum_local_information = 0;
		share_data->sum_global_information = 0;
		share_data->sum_robot_cost = 0;
		for (int i = 0; i < now_view_space->views.size(); i++) {
			share_data->sum_local_information += now_view_space->views[i].information_gain;
			share_data->sum_global_information += now_view_space->views[i].get_global_information();
			share_data->sum_robot_cost += now_view_space->views[i].robot_cost;
		}
		if (share_data->sum_local_information == 0) cout << "full information is zero." << endl;
		for (int i = 0; i < now_view_space->views.size(); i++) {
			if (share_data->move_cost_on == false) {
				if (share_data->method_of_IG == MCMF) now_view_space->views[i].final_utility = (1 - share_data->cost_weight) * now_view_space->views[i].information_gain / share_data->sum_local_information + share_data->cost_weight * now_view_space->views[i].get_global_information() / share_data->sum_global_information;
				else now_view_space->views[i].final_utility = now_view_space->views[i].information_gain;
			}
			else {
				if (share_data->method_of_IG == MCMF) now_view_space->views[i].final_utility = (1 - share_data->move_weight)* ((1 - share_data->cost_weight) * now_view_space->views[i].information_gain / share_data->sum_local_information + share_data->cost_weight * now_view_space->views[i].get_global_information() / share_data->sum_global_information) + share_data->move_weight * (share_data->robot_cost_negtive == true ? -1 : 1) * now_view_space->views[i].robot_cost / share_data->sum_robot_cost;
				else now_view_space->views[i].final_utility = (1 - share_data->move_weight) * now_view_space->views[i].information_gain + share_data->move_weight * (share_data->robot_cost_negtive == true ? -1 : 1) * now_view_space->views[i].robot_cost / share_data->sum_robot_cost;
			}
		}
	}
	//���±�־λ
	share_data->now_views_infromation_processed = true;
}

void move_robot(View* now_best_view, View_Space* now_view_space, Share_Data* share_data) {
	if (share_data->num_of_max_iteration > 0 && nbv_plan->iterations + 1 >= share_data->num_of_max_iteration) share_data->over = true;
	if (!share_data->move_wait) share_data->move_on = true;
}

void show_cloud(pcl::visualization::PCLVisualizer::Ptr viewer) {
	//pcl��ʾ����
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

void get_command()
{	//�ӿ���̨��ȡָ���ַ���
	string cmd;
	while (!stop && !share_data->over)
	{
		cout << "Input command 1.stop 2.over 3.next_itreation :" << endl;
		cin >> cmd;
		if (cmd == "1") stop = true;
		else if (cmd == "2") share_data->over = true;
		else if (cmd == "3") share_data->move_on = true;
		else cout << "Wrong command.Retry :" << endl;
	}
	cout << "get_command over." << endl;
}

void get_run()
{
	//NBV�滮�ڳ�ʼ��
	nbv_plan = new NBV_Planner(share_data);
	//����ѭ��
	string status="";
	//ʵʱ��ȡ��滮
	while (!stop && nbv_plan->plan()) {
		//���״̬�б仯�����
		if (status != nbv_plan->out_status()) {
			status = nbv_plan->out_status();
			cout << "NBV_Planner's status is " << status << endl;
		}
	}
	delete nbv_plan;
}

#define DebugOne 0
#define TestAll 1

int mode = DebugOne;

int main()
{
	//Init
	ios::sync_with_stdio(false);
	//ѡȡģʽ
	if (mode == DebugOne)
	{
		//��������ʼ��
		share_data = new Share_Data("../DefaultConfiguration.yaml");
		//����̨��ȡָ���߳�
		thread cmd(get_command);
		//NBVϵͳ�����߳�
		thread runner(get_run);
		//�ȴ��߳̽���
		runner.join();
		cmd.join();
		delete share_data;
	}
	else if (mode == TestAll){
		//���Լ�
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
		//����
		for (int i = 0; i < names.size(); i++) {
			//��������ʼ��
			share_data = new Share_Data("../DefaultConfiguration.yaml", names[i]);
			//NBVϵͳ�����߳�
			thread runner(get_run);
			//�ȴ��߳̽���
			runner.join();
			delete share_data;
		}
	}
	cout << "System over." << endl;
	return 0;
}
