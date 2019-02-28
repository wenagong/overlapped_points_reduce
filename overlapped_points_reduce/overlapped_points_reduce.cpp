#include "stdafx.h"
#include "overlapped_points_reduce.h"
#include<iostream>
#include<fstream>
#include<string>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/registration/correspondence_estimation.h>
#include<pcl/kdtree/io.h>
#include<pcl/features/normal_3d.h>
#include<pcl/filters/extract_indices.h>
#include<pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include<pcl/segmentation/sac_segmentation.h>
#include<pcl/segmentation/extract_clusters.h>
#include <pcl/surface/mls.h> //MLS最小二乘
#include<opencv.hpp>

using namespace std;
using namespace cv;

#define Lcx 652.74           
#define Lcy 459.102    //左相机的光心位置

#define Rcx 610.779
#define Rcy 445.699   //右相机的光心位置

#define fx1 3359.59
#define fy1 3360.56   //左相机的焦距

#define fx2 3369.11
#define fy2 3369.67   //右相机的焦距

#define La1 -0.0584155
#define La2 0.532731
#define La3 0.000793009	
#define La4 -0.00217376   //左相机的四个畸变参数

#define Ra1 -0.0212227
#define Ra2 0.5244
#define Ra3  0.000552162
#define Ra4 -0.00357077  //右相机的四个畸变参数

#define C_MAX 13  //相机改变位置最大次数


//自定义点类型结构，包括三维坐标和对应的二维坐标
struct MyPointType {
	PCL_ADD_POINT4D;
	double p_x;   //对应二维坐标
	double p_y;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGEN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(MyPointType,  //注册点类型宏
(double, x, x)
(double, y, y)
(double, z, z)
(double, p_x, p_x)
(double, p_y, p_y)
)


overlapped_points_reduce::overlapped_points_reduce(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
}

void overlapped_points_reduce::OnGetPcd() {   //点云转换为PCD格式
	//ui.pushButton_1->setText(tr("(OnGetPcd)"));
	for (int i = 1; i <= C_MAX; i++) {

		//读取三维点云txt文件
		char data_name[40];
		sprintf_s(data_name, "%s%d%s", "datas\\", i, "\\NewResult3D_Points.txt");
		ifstream file_name;
		file_name.open(data_name, ios::in); //ios::in 表示以只读的方式读入文件

		//读取二维匹配点坐标txt文件
		char TD_name[40];
		sprintf_s(TD_name, "%s%d%s", "datas\\", i, "\\MatchResult2D_LeftPoints.txt");
		ifstream match_name;
		match_name.open(TD_name, ios::in);

		//点云文件打开失败
		if (!file_name.is_open()) {
			cout << "open" << i << "3D datas error!" << endl;
		}

		//匹配点坐标文件打开失败
		if (!match_name.is_open()) {
			cout << "open" << i << "2D datas error!" << endl;
		}

		//求点云总数目rows
		int rows = -1;
		char str[200];
		while (!file_name.eof()) {  //end of file“文件结束"，用来判断是否到达文件结尾
			file_name.getline(str, sizeof(str)); 
			rows++;
		}

		file_name.clear();
		file_name.seekg(0, ios::beg);//把文件的读指针从文件开头向后移0个字节(即指针指向开头的位置)

		pcl::PointCloud<pcl::PointXYZ>cloud;
		cloud.width = rows;
		cloud.height = 1;
		cloud.is_dense = false;  //判断点云中的点是否包含 Inf/NaN这种值（包含为false）										  
		cloud.points.resize(cloud.width*cloud.height);

		pcl::PointCloud<MyPointType>m_cloud;
		m_cloud.width = rows;
		m_cloud.height = 1;
		m_cloud.is_dense = false;
		m_cloud.resize(m_cloud.width*m_cloud.height);

		//txt数据写入点云文件
		for (int i = 0; i < rows; i++) {

			double num[5];
			//file_name.setf(ios::fixed, ios::floatfield);   //设置小数精度
			//file_name.precision(7);

			file_name >> num[0];  //按txt行输入
			file_name >> num[1];
			file_name >> num[2];

			match_name >> num[3];
			match_name >> num[4];

			cloud.points[i].x = num[0];
			cloud.points[i].y = num[1];
			cloud.points[i].z = num[2];

			m_cloud.points[i].x = num[0];
			m_cloud.points[i].y = num[1];
			m_cloud.points[i].z = num[2];
			m_cloud.points[i].p_x = num[3];
			m_cloud.points[i].p_y = num[4];
		}

		////摄站+投影仪位命名
		//int A = i / 2 + 1;   //以摄像机位置命名
		//int B;
		//if (i == 1)B = 1;   //每个位置下第几次投影位置
		//else B = i % 2 + 1;

		//char cloud_name[40];
		//sprintf_s(cloud_name, "%s%d%d%s", "pcd_datas\\cloud", A, B, ".pcd");    //点云命名 

		//char cloud1_name[40];
		//sprintf_s(cloud1_name, "%s%d%d%s", "pcd_datas\\m_cloud", A, B, ".pcd");    //点云命名 

		//以次数命名
		char cloud_name[40];
		sprintf_s(cloud_name, "%s%d%s", "pcd_datas\\cloud", i, ".pcd");    //点云命名 

		char cloud1_name[40];
		sprintf_s(cloud1_name, "%s%d%s", "pcd_datas\\m_cloud", i, ".pcd");    //点云命名 

		pcl::io::savePCDFileASCII(cloud_name, cloud);
		pcl::io::savePCDFileASCII(cloud1_name, m_cloud);

		file_name, match_name.close();
	}
	QMessageBox msg1;
	msg1.setText("Get PCD Succeed ! you can close it ! ");
	msg1.exec();
}

/*****************计算相邻摄站重叠区域点云************************/

void overlapped_points_reduce::CalculateOverlap() {

	for (int p = 1; p < C_MAX; p++) {

		int q = p + 1;

		char cloud_name1[30];
		sprintf_s(cloud_name1, "%s%d%s", "pcd_datas\\cloud", p, ".pcd");

		char cloud_name2[30];
		sprintf_s(cloud_name2, "%s%d%s", "pcd_datas\\cloud", q, ".pcd");

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::io::loadPCDFile<pcl::PointXYZ>(cloud_name1, *cloudA);

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::io::loadPCDFile<pcl::PointXYZ>(cloud_name2, *cloudB);

		//求出相邻点云的近邻点索引
		pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ>core;
		core.setInputSource(cloudA);
		core.setInputTarget(cloudB);

		boost::shared_ptr<pcl::Correspondences>cor(new pcl::Correspondences);

		core.determineReciprocalCorrespondences(*cor, 0.8);  //点之间的最大距离

		//定义重叠点云格式
		pcl::PointCloud<pcl::PointXYZ>overlap1;
		overlap1.width = cor->size();
		overlap1.height = 1;
		overlap1.is_dense = false;
		overlap1.resize(overlap1.width*overlap1.height);

		//定义重叠点云格式
		pcl::PointCloud<pcl::PointXYZ>overlap2;
		overlap2.width = cor->size();
		overlap2.height = 1;
		overlap2.is_dense = false;
		overlap2.resize(overlap2.width*overlap2.height);


		for (size_t i = 0; i < cor->size(); i++) { //遍历所有索引

			overlap1.points[i].x = cloudA->points[cor->at(i).index_query].x;//source
			overlap1.points[i].y = cloudA->points[cor->at(i).index_query].y;
			overlap1.points[i].z = cloudA->points[cor->at(i).index_query].z;

			overlap2.points[i].x = cloudB->points[cor->at(i).index_match].x;
			overlap2.points[i].y = cloudB->points[cor->at(i).index_match].y;
			overlap2.points[i].z = cloudB->points[cor->at(i).index_match].z;
		}
		//相邻摄站下两次的重叠点云
		char overlap_name1[40];
		sprintf_s(overlap_name1, "%s%d%s", "overlap_datas\\overlap", p, "-1.pcd");
		pcl::io::savePCDFileASCII(overlap_name1, overlap1);

		char overlap_name2[40];
		sprintf_s(overlap_name2, "%s%d%s", "overlap_datas\\overlap", p, "-2.pcd");
		pcl::io::savePCDFileASCII(overlap_name2, overlap2);
	}
	QMessageBox msg2;
	msg2.setText("Calculate OverlappedPoints Succeed ! you can close it ! ");
	msg2.exec();
}


/******************消除相邻摄站测量的重叠点云，得到最后精简结果*************************/

void overlapped_points_reduce::GetFinalResult() {
	ofstream debug;
	debug.open("debug.txt");
	//相机参数
	double m1[3][3] = { { fx1,0,Lcx },{ 0,fy1,Lcy },{ 0,0,1 } };
	Mat m1_matrix(Size(3, 3), CV_64F, m1);  //左相机的内参矩阵

	double r[3][1] = { { -0.0180031 },{ 0.0973247 },{ -0.04762 } };
	Mat R(Size(1, 3), CV_64F, r); //旋转向量

	Mat T = (Mat_<double>(3, 1) << -92.9037, 3.04819, 6.23956);   //平移向量

	Mat dist1 = (Mat_<double>(1, 4) << La1, La2, La3, La4); //左相机畸变参数(k1,k2,p1,p2,k3)

	for (int p = 1; p < C_MAX; p++) {
		
		//相邻摄站索引
		int q = p + 1;

		char cloud_name1[30];
		sprintf_s(cloud_name1, "%s%d%s", "final_datas\\cloud", p, ".pcd");

		char cloud_name2[30];
		sprintf_s(cloud_name2, "%s%d%s", "final_datas\\cloud", q, ".pcd");

		char match_name1[30];
		sprintf_s(match_name1, "%s%d%s", "pcd_datas\\m_cloud", p, ".pcd");

		char match_name2[30];
		sprintf_s(match_name2, "%s%d%s", "pcd_datas\\m_cloud", q, ".pcd");

		//如果在final_datas里面找不到文件就去初始目录pcd_dats里面查找（避免重复去冗）
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA(new pcl::PointCloud<pcl::PointXYZ>);

		if (pcl::io::loadPCDFile<pcl::PointXYZ>(cloud_name1, *cloudA) == -1) {  //读取失败
			
			char init_name1[30];
			sprintf_s(init_name1, "%s%d%s", "pcd_datas\\cloud", p, ".pcd");  //初始pcd文件目录

			pcl::io::loadPCDFile<pcl::PointXYZ>(init_name1, *cloudA);
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB(new pcl::PointCloud<pcl::PointXYZ>);

		if (pcl::io::loadPCDFile<pcl::PointXYZ>(cloud_name2, *cloudB) == -1) {

			char init_name2[30];
			sprintf_s(init_name2, "%s%d%s", "pcd_datas\\cloud", q, ".pcd");

			pcl::io::loadPCDFile<pcl::PointXYZ>(init_name2, *cloudB);
		}

		pcl::PointCloud<MyPointType>m_cloudA;
		pcl::io::loadPCDFile<MyPointType>(match_name1, m_cloudA);

		pcl::PointCloud<MyPointType>m_cloudB;
		pcl::io::loadPCDFile<MyPointType>(match_name2, m_cloudB);

		pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ>core;
		core.setInputSource(cloudA);
		core.setInputTarget(cloudB);

		boost::shared_ptr<pcl::Correspondences>cor(new pcl::Correspondences);

		core.determineReciprocalCorrespondences(*cor, 0.8);  //点之间的最大距离

		//定义重叠点云格式
		pcl::PointCloud<pcl::PointXYZ>overlap;
		overlap.width = cor->size();
		overlap.height = 1;
		overlap.is_dense = false;
		overlap.resize(overlap.width*overlap.height);

		for (size_t i = 0; i < cor->size(); i++) { //遍历所有索引

			//double coord_pointsA[3]; //三维点坐标
			//double coord_pointsB[3];
			vector<Point3d>coord_pointsA;
			vector<Point3d>coord_pointsB;

			//double res_2dA[2];  //生成的投影点二维坐标
			//double res_2dB[2];  
			vector<Point2d>res_2dA;
			vector<Point2d>res_2dB;

			//匹配点坐标
			double match1[2];
			double match2[2];

			match1[0] = m_cloudA.points[cor->at(i).index_query].p_x;
			match1[1] = m_cloudA.points[cor->at(i).index_query].p_y;

			match2[0] = m_cloudB.points[cor->at(i).index_match].p_x;
			match2[1] = m_cloudB.points[cor->at(i).index_match].p_y;

			Mat cordA = (Mat_<double>(1, 3) << cloudA->points[cor->at(i).index_query].x, cloudA->points[cor->at(i).index_query].y, cloudA->points[cor->at(i).index_query].z);
			coord_pointsA = Mat_<Point3d>(cordA);

			Mat cordB = (Mat_<double>(1, 3) << cloudB->points[cor->at(i).index_match].x, cloudB->points[cor->at(i).index_match].y, cloudB->points[cor->at(i).index_match].z);
			coord_pointsB = Mat_<Point3d>(cordB);

			//coord_pointsA[0] = cloudA->points[cor->at(i).index_query].x;
			//coord_pointsA[1] = cloudA->points[cor->at(i).index_query].y;
			//coord_pointsA[2] = cloudA->points[cor->at(i).index_query].z;

			//coord_pointsB[0] = cloudB->points[cor->at(i).index_match].x;
			//coord_pointsB[1] = cloudB->points[cor->at(i).index_match].y;
			//coord_pointsB[2] = cloudB->points[cor->at(i).index_match].z;

			//计算重投影坐标
			projectPoints(coord_pointsA, R, T, m1_matrix, dist1, res_2dA);
			projectPoints(coord_pointsB, R, T, m1_matrix, dist1, res_2dB);

			//重投影误差大小
			double dev1;
			double dev2;
			dev1 = pow(res_2dA.at(0).x - match1[0], 2) + pow(res_2dA.at(0).y - match1[1], 2);
			dev2 = pow(res_2dB.at(0).x - match2[0], 2) + pow(res_2dB.at(0).y - match2[1], 2);
			
			//if(1000<i<5000)  debug << "dev1=" << dev1 << "  " << "dev2=" << dev2 << endl;

			if (dev1 < dev2) {
				overlap.points[i].x = cloudA->points[cor->at(i).index_query].x;
				overlap.points[i].y = cloudA->points[cor->at(i).index_query].y;
				overlap.points[i].z = cloudA->points[cor->at(i).index_query].z;

				cloudB->points[cor->at(i).index_match].x = NULL;
				cloudB->points[cor->at(i).index_match].y = NULL;
				cloudB->points[cor->at(i).index_match].z = NULL;

			}
			else {
				overlap.points[i].x = cloudB->points[cor->at(i).index_match].x;
				overlap.points[i].y = cloudB->points[cor->at(i).index_match].y;
				overlap.points[i].z = cloudB->points[cor->at(i).index_match].z;

				cloudA->points[cor->at(i).index_query].x = NULL;
				cloudA->points[cor->at(i).index_query].y = NULL;
				cloudA->points[cor->at(i).index_query].z = NULL;

			}

			////从AB中去除索引点部分 （这样会导致保留下来的重叠点无法继续与后面的点云做配准）
			//cloudA->points[cor->at(i).index_query].x = NULL;
			//cloudA->points[cor->at(i).index_query].y = NULL;
			//cloudA->points[cor->at(i).index_query].z = NULL;

			//cloudB->points[cor->at(i).index_match].x = NULL;
			//cloudB->points[cor->at(i).index_match].y = NULL;
			//cloudB->points[cor->at(i).index_match].z = NULL;

		}

		//for (size_t k = 0; k < cor->size(); k++) {
		//	
		//	//从点云A,B中去除重叠点
		//	pcl::PointCloud<pcl::PointXYZ>::iterator index1 = cloudA->begin();
		//	cloudA->erase(index1 + cor->at(k).index_query);

		//	pcl::PointCloud<pcl::PointXYZ>::iterator index2 = cloudB->begin();
		//	cloudB->erase(index2 + cor->at(k).index_match);
		//}

		char overlap_name[40];
		sprintf_s(overlap_name, "%s%d%s", "final_datas\\overlap", p, ".pcd");
		pcl::io::savePCDFileASCII(overlap_name, overlap);

		char file_name1[30];
		sprintf_s(file_name1, "%s%d%s", "final_datas\\cloud", p, ".pcd");
		pcl::io::savePCDFileASCII(file_name1, *cloudA);

		char file_name2[30];
		sprintf_s(file_name2, "%s%d%s", "final_datas\\cloud", q, ".pcd");
		pcl::io::savePCDFileASCII(file_name2, *cloudB);
	}

	QMessageBox msg3;
	msg3.setText("Get Result Succeed ! you can close it ! ");
	msg3.exec();
}


/****************合并零散点云，生成一个整体点云数据******************/

void overlapped_points_reduce::GetWholeDatas() {
    
	//合并求得的所有的单个摄站的重叠点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_overlap(new pcl::PointCloud<pcl::PointXYZ>);   //存储所有的重叠部分点云
	for (int i = 1; i < C_MAX; i++) {
			pcl::PointCloud<pcl::PointXYZ>::Ptr overlap_tmp(new pcl::PointCloud<pcl::PointXYZ>);
			char overlap_file_path[40];
			sprintf_s(overlap_file_path,"%s%d%s", "final_datas\\overlap", i, ".pcd");;
			pcl::io::loadPCDFile(overlap_file_path, *overlap_tmp); //读取每一部分重叠区域点云
			*cloud_overlap = *cloud_overlap + *overlap_tmp;
			overlap_tmp->clear();
	}

	//合并所有除去重叠区域的点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_reduce(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 1; i <= C_MAX; i++) {
			pcl::PointCloud<pcl::PointXYZ>::Ptr ruduce_tmp(new pcl::PointCloud<pcl::PointXYZ>);
			char ruduce_file_path[40];
			sprintf_s(ruduce_file_path, "%s%d%s", "final_datas\\cloud", i, ".pcd");;
			pcl::io::loadPCDFile(ruduce_file_path, *ruduce_tmp); //读取每一部分重叠区域点云
			*cloud_reduce = *cloud_reduce + *ruduce_tmp;
			ruduce_tmp->clear();
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_whole(new pcl::PointCloud<pcl::PointXYZ>);
	*cloud_whole =  *cloud_overlap + *cloud_reduce;

	pcl::io::savePCDFileASCII("final_datas\\whole_datas.pcd", *cloud_whole);

	QMessageBox msg4;
	msg4.setText("Get Result Succeed ! you can close it ! ");
	msg4.exec();
}


/*************************去除背景冗余点云*******************************/

void overlapped_points_reduce::RemoveBackgroundDatas(){

	pcl::PointCloud<pcl::PointXYZ>::Ptr whole_datas(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("final_datas\\whole_datas.pcd", *whole_datas);   //读取完整点云
	 
	//// Scheme1：半径滤波（无法分割大块背景点云效果，可以去除整体点云附加的一些离群点）  
	//pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
	//outrem.setInputCloud(whole_datas);
	//outrem.setRadiusSearch(3);  //搜索邻近点的范围大小
	//outrem.setMinNeighborsInRadius(100);  //设置查询点的邻近点集数小于X的删除

	//pcl::PointCloud<pcl::PointXYZ>::Ptr filter_radius(new pcl::PointCloud<pcl::PointXYZ>);
	//outrem.filter(*filter_radius);

	//pcl::io::savePCDFileASCII("filter_datas\\filter_radius.pcd", *filter_radius);
	
	//下采样
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_down(new pcl::PointCloud<pcl::PointXYZ>);
	vg.setInputCloud(whole_datas);
	vg.setLeafSize(1.4f, 1.4f, 1.4f); //使用Xcm长的叶子节点大小进行下采样
	vg.filter(*cloud_down);

	pcl::io::savePCDFile("filter_datas\\cloud_down.pcd", *cloud_down);

	// Scheme2：欧式分割
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	kdtree->setInputCloud(cloud_down); // 创建点云索引向量，用于存储实际的点云信息

	vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec; //欧式聚类提取
	ec.setClusterTolerance(4);  //近邻搜索半径Xm，值越大保留的越多，4.0单个效果最好
	ec.setMinClusterSize(10000);  //设置一个聚类需要最少点为25000
	//ec.setMaxClusterSize(10000);
	ec.setSearchMethod(kdtree);  //设置点云的索引机制
	ec.setInputCloud(cloud_down);
	ec.extract(cluster_indices); //从点云中提取聚类，并将点云索引保存在cluster_indices中

	/*为了从点云索引向量中分割出每个聚类，必须迭代访问点云索引，每次创建一个新的点云数据集，
	并且将所有当前聚类的点写入到点云数据集中。*/
	//迭代访问点云索引cluster_indices，直到分割出所有聚类

	int j = 0;
	for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		//创建新的点云数据集cloud_cluster，将所有当前聚类写入到点云数据集中
		for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
			cloud_cluster->points.push_back(cloud_down->points[*pit]);
		}
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		
		stringstream ss;
		ss << "filter_datas\\cloud_clouster_" << j << ".pcd"; 
		pcl::io::savePCDFileASCII(ss.str(), *cloud_cluster);
		j++;
	}


	QMessageBox msg5;
	msg5.setText("Get Result Succeed ! you can close it ! ");
	msg5.exec();
}

/************************滤波去噪*************************/
void overlapped_points_reduce::FilterDenoise() {    //MLS滑动最小二乘法平滑滤波
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("final_datas\\cloud11.pcd", *cloud); 
	
    // Create a KD-Tree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

	// Output has the PointNormal type in order to store the normals calculated by MLS
	pcl::PointCloud<pcl::PointNormal> mls_points;

	// Init object (second point type is for the normals, even if unused)
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

	mls.setComputeNormals(true); //是否输出法向量

	// Set parameters
	mls.setInputCloud(cloud);
	mls.setPolynomialFit(true);
	mls.setPolynomialOrder(2);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(0.015);

	// Reconstruct
	mls.process(mls_points);

	// Save output
	pcl::io::savePCDFile("filter_datas\\MLS.pcd", mls_points);
	//pcl::io::savePCDFile("MLS.pcd", mls_points);

	QMessageBox msg;
	msg.setText("Get Result Succeed ! you can close it ! ");
	msg.exec();

}
