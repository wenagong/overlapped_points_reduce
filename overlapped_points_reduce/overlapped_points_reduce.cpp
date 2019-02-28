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
#include <pcl/surface/mls.h> //MLS��С����
#include<opencv.hpp>

using namespace std;
using namespace cv;

#define Lcx 652.74           
#define Lcy 459.102    //������Ĺ���λ��

#define Rcx 610.779
#define Rcy 445.699   //������Ĺ���λ��

#define fx1 3359.59
#define fy1 3360.56   //������Ľ���

#define fx2 3369.11
#define fy2 3369.67   //������Ľ���

#define La1 -0.0584155
#define La2 0.532731
#define La3 0.000793009	
#define La4 -0.00217376   //��������ĸ��������

#define Ra1 -0.0212227
#define Ra2 0.5244
#define Ra3  0.000552162
#define Ra4 -0.00357077  //��������ĸ��������

#define C_MAX 13  //����ı�λ��������


//�Զ�������ͽṹ��������ά����Ͷ�Ӧ�Ķ�ά����
struct MyPointType {
	PCL_ADD_POINT4D;
	double p_x;   //��Ӧ��ά����
	double p_y;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGEN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(MyPointType,  //ע������ͺ�
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

void overlapped_points_reduce::OnGetPcd() {   //����ת��ΪPCD��ʽ
	//ui.pushButton_1->setText(tr("(OnGetPcd)"));
	for (int i = 1; i <= C_MAX; i++) {

		//��ȡ��ά����txt�ļ�
		char data_name[40];
		sprintf_s(data_name, "%s%d%s", "datas\\", i, "\\NewResult3D_Points.txt");
		ifstream file_name;
		file_name.open(data_name, ios::in); //ios::in ��ʾ��ֻ���ķ�ʽ�����ļ�

		//��ȡ��άƥ�������txt�ļ�
		char TD_name[40];
		sprintf_s(TD_name, "%s%d%s", "datas\\", i, "\\MatchResult2D_LeftPoints.txt");
		ifstream match_name;
		match_name.open(TD_name, ios::in);

		//�����ļ���ʧ��
		if (!file_name.is_open()) {
			cout << "open" << i << "3D datas error!" << endl;
		}

		//ƥ��������ļ���ʧ��
		if (!match_name.is_open()) {
			cout << "open" << i << "2D datas error!" << endl;
		}

		//���������Ŀrows
		int rows = -1;
		char str[200];
		while (!file_name.eof()) {  //end of file���ļ�����"�������ж��Ƿ񵽴��ļ���β
			file_name.getline(str, sizeof(str)); 
			rows++;
		}

		file_name.clear();
		file_name.seekg(0, ios::beg);//���ļ��Ķ�ָ����ļ���ͷ�����0���ֽ�(��ָ��ָ��ͷ��λ��)

		pcl::PointCloud<pcl::PointXYZ>cloud;
		cloud.width = rows;
		cloud.height = 1;
		cloud.is_dense = false;  //�жϵ����еĵ��Ƿ���� Inf/NaN����ֵ������Ϊfalse��										  
		cloud.points.resize(cloud.width*cloud.height);

		pcl::PointCloud<MyPointType>m_cloud;
		m_cloud.width = rows;
		m_cloud.height = 1;
		m_cloud.is_dense = false;
		m_cloud.resize(m_cloud.width*m_cloud.height);

		//txt����д������ļ�
		for (int i = 0; i < rows; i++) {

			double num[5];
			//file_name.setf(ios::fixed, ios::floatfield);   //����С������
			//file_name.precision(7);

			file_name >> num[0];  //��txt������
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

		////��վ+ͶӰ��λ����
		//int A = i / 2 + 1;   //�������λ������
		//int B;
		//if (i == 1)B = 1;   //ÿ��λ���µڼ���ͶӰλ��
		//else B = i % 2 + 1;

		//char cloud_name[40];
		//sprintf_s(cloud_name, "%s%d%d%s", "pcd_datas\\cloud", A, B, ".pcd");    //�������� 

		//char cloud1_name[40];
		//sprintf_s(cloud1_name, "%s%d%d%s", "pcd_datas\\m_cloud", A, B, ".pcd");    //�������� 

		//�Դ�������
		char cloud_name[40];
		sprintf_s(cloud_name, "%s%d%s", "pcd_datas\\cloud", i, ".pcd");    //�������� 

		char cloud1_name[40];
		sprintf_s(cloud1_name, "%s%d%s", "pcd_datas\\m_cloud", i, ".pcd");    //�������� 

		pcl::io::savePCDFileASCII(cloud_name, cloud);
		pcl::io::savePCDFileASCII(cloud1_name, m_cloud);

		file_name, match_name.close();
	}
	QMessageBox msg1;
	msg1.setText("Get PCD Succeed ! you can close it ! ");
	msg1.exec();
}

/*****************����������վ�ص��������************************/

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

		//������ڵ��ƵĽ��ڵ�����
		pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ>core;
		core.setInputSource(cloudA);
		core.setInputTarget(cloudB);

		boost::shared_ptr<pcl::Correspondences>cor(new pcl::Correspondences);

		core.determineReciprocalCorrespondences(*cor, 0.8);  //��֮���������

		//�����ص����Ƹ�ʽ
		pcl::PointCloud<pcl::PointXYZ>overlap1;
		overlap1.width = cor->size();
		overlap1.height = 1;
		overlap1.is_dense = false;
		overlap1.resize(overlap1.width*overlap1.height);

		//�����ص����Ƹ�ʽ
		pcl::PointCloud<pcl::PointXYZ>overlap2;
		overlap2.width = cor->size();
		overlap2.height = 1;
		overlap2.is_dense = false;
		overlap2.resize(overlap2.width*overlap2.height);


		for (size_t i = 0; i < cor->size(); i++) { //������������

			overlap1.points[i].x = cloudA->points[cor->at(i).index_query].x;//source
			overlap1.points[i].y = cloudA->points[cor->at(i).index_query].y;
			overlap1.points[i].z = cloudA->points[cor->at(i).index_query].z;

			overlap2.points[i].x = cloudB->points[cor->at(i).index_match].x;
			overlap2.points[i].y = cloudB->points[cor->at(i).index_match].y;
			overlap2.points[i].z = cloudB->points[cor->at(i).index_match].z;
		}
		//������վ�����ε��ص�����
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


/******************����������վ�������ص����ƣ��õ���󾫼���*************************/

void overlapped_points_reduce::GetFinalResult() {
	ofstream debug;
	debug.open("debug.txt");
	//�������
	double m1[3][3] = { { fx1,0,Lcx },{ 0,fy1,Lcy },{ 0,0,1 } };
	Mat m1_matrix(Size(3, 3), CV_64F, m1);  //��������ڲξ���

	double r[3][1] = { { -0.0180031 },{ 0.0973247 },{ -0.04762 } };
	Mat R(Size(1, 3), CV_64F, r); //��ת����

	Mat T = (Mat_<double>(3, 1) << -92.9037, 3.04819, 6.23956);   //ƽ������

	Mat dist1 = (Mat_<double>(1, 4) << La1, La2, La3, La4); //������������(k1,k2,p1,p2,k3)

	for (int p = 1; p < C_MAX; p++) {
		
		//������վ����
		int q = p + 1;

		char cloud_name1[30];
		sprintf_s(cloud_name1, "%s%d%s", "final_datas\\cloud", p, ".pcd");

		char cloud_name2[30];
		sprintf_s(cloud_name2, "%s%d%s", "final_datas\\cloud", q, ".pcd");

		char match_name1[30];
		sprintf_s(match_name1, "%s%d%s", "pcd_datas\\m_cloud", p, ".pcd");

		char match_name2[30];
		sprintf_s(match_name2, "%s%d%s", "pcd_datas\\m_cloud", q, ".pcd");

		//�����final_datas�����Ҳ����ļ���ȥ��ʼĿ¼pcd_dats������ң������ظ�ȥ�ߣ�
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA(new pcl::PointCloud<pcl::PointXYZ>);

		if (pcl::io::loadPCDFile<pcl::PointXYZ>(cloud_name1, *cloudA) == -1) {  //��ȡʧ��
			
			char init_name1[30];
			sprintf_s(init_name1, "%s%d%s", "pcd_datas\\cloud", p, ".pcd");  //��ʼpcd�ļ�Ŀ¼

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

		core.determineReciprocalCorrespondences(*cor, 0.8);  //��֮���������

		//�����ص����Ƹ�ʽ
		pcl::PointCloud<pcl::PointXYZ>overlap;
		overlap.width = cor->size();
		overlap.height = 1;
		overlap.is_dense = false;
		overlap.resize(overlap.width*overlap.height);

		for (size_t i = 0; i < cor->size(); i++) { //������������

			//double coord_pointsA[3]; //��ά������
			//double coord_pointsB[3];
			vector<Point3d>coord_pointsA;
			vector<Point3d>coord_pointsB;

			//double res_2dA[2];  //���ɵ�ͶӰ���ά����
			//double res_2dB[2];  
			vector<Point2d>res_2dA;
			vector<Point2d>res_2dB;

			//ƥ�������
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

			//������ͶӰ����
			projectPoints(coord_pointsA, R, T, m1_matrix, dist1, res_2dA);
			projectPoints(coord_pointsB, R, T, m1_matrix, dist1, res_2dB);

			//��ͶӰ����С
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

			////��AB��ȥ�������㲿�� �������ᵼ�±����������ص����޷����������ĵ�������׼��
			//cloudA->points[cor->at(i).index_query].x = NULL;
			//cloudA->points[cor->at(i).index_query].y = NULL;
			//cloudA->points[cor->at(i).index_query].z = NULL;

			//cloudB->points[cor->at(i).index_match].x = NULL;
			//cloudB->points[cor->at(i).index_match].y = NULL;
			//cloudB->points[cor->at(i).index_match].z = NULL;

		}

		//for (size_t k = 0; k < cor->size(); k++) {
		//	
		//	//�ӵ���A,B��ȥ���ص���
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


/****************�ϲ���ɢ���ƣ�����һ�������������******************/

void overlapped_points_reduce::GetWholeDatas() {
    
	//�ϲ���õ����еĵ�����վ���ص�����
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_overlap(new pcl::PointCloud<pcl::PointXYZ>);   //�洢���е��ص����ֵ���
	for (int i = 1; i < C_MAX; i++) {
			pcl::PointCloud<pcl::PointXYZ>::Ptr overlap_tmp(new pcl::PointCloud<pcl::PointXYZ>);
			char overlap_file_path[40];
			sprintf_s(overlap_file_path,"%s%d%s", "final_datas\\overlap", i, ".pcd");;
			pcl::io::loadPCDFile(overlap_file_path, *overlap_tmp); //��ȡÿһ�����ص��������
			*cloud_overlap = *cloud_overlap + *overlap_tmp;
			overlap_tmp->clear();
	}

	//�ϲ����г�ȥ�ص�����ĵ���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_reduce(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 1; i <= C_MAX; i++) {
			pcl::PointCloud<pcl::PointXYZ>::Ptr ruduce_tmp(new pcl::PointCloud<pcl::PointXYZ>);
			char ruduce_file_path[40];
			sprintf_s(ruduce_file_path, "%s%d%s", "final_datas\\cloud", i, ".pcd");;
			pcl::io::loadPCDFile(ruduce_file_path, *ruduce_tmp); //��ȡÿһ�����ص��������
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


/*************************ȥ�������������*******************************/

void overlapped_points_reduce::RemoveBackgroundDatas(){

	pcl::PointCloud<pcl::PointXYZ>::Ptr whole_datas(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("final_datas\\whole_datas.pcd", *whole_datas);   //��ȡ��������
	 
	//// Scheme1���뾶�˲����޷��ָ��鱳������Ч��������ȥ��������Ƹ��ӵ�һЩ��Ⱥ�㣩  
	//pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
	//outrem.setInputCloud(whole_datas);
	//outrem.setRadiusSearch(3);  //�����ڽ���ķ�Χ��С
	//outrem.setMinNeighborsInRadius(100);  //���ò�ѯ����ڽ��㼯��С��X��ɾ��

	//pcl::PointCloud<pcl::PointXYZ>::Ptr filter_radius(new pcl::PointCloud<pcl::PointXYZ>);
	//outrem.filter(*filter_radius);

	//pcl::io::savePCDFileASCII("filter_datas\\filter_radius.pcd", *filter_radius);
	
	//�²���
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_down(new pcl::PointCloud<pcl::PointXYZ>);
	vg.setInputCloud(whole_datas);
	vg.setLeafSize(1.4f, 1.4f, 1.4f); //ʹ��Xcm����Ҷ�ӽڵ��С�����²���
	vg.filter(*cloud_down);

	pcl::io::savePCDFile("filter_datas\\cloud_down.pcd", *cloud_down);

	// Scheme2��ŷʽ�ָ�
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	kdtree->setInputCloud(cloud_down); // ���������������������ڴ洢ʵ�ʵĵ�����Ϣ

	vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec; //ŷʽ������ȡ
	ec.setClusterTolerance(4);  //���������뾶Xm��ֵԽ������Խ�࣬4.0����Ч�����
	ec.setMinClusterSize(10000);  //����һ��������Ҫ���ٵ�Ϊ25000
	//ec.setMaxClusterSize(10000);
	ec.setSearchMethod(kdtree);  //���õ��Ƶ���������
	ec.setInputCloud(cloud_down);
	ec.extract(cluster_indices); //�ӵ�������ȡ���࣬������������������cluster_indices��

	/*Ϊ�˴ӵ������������зָ��ÿ�����࣬����������ʵ���������ÿ�δ���һ���µĵ������ݼ���
	���ҽ����е�ǰ����ĵ�д�뵽�������ݼ��С�*/
	//�������ʵ�������cluster_indices��ֱ���ָ�����о���

	int j = 0;
	for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		//�����µĵ������ݼ�cloud_cluster�������е�ǰ����д�뵽�������ݼ���
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

/************************�˲�ȥ��*************************/
void overlapped_points_reduce::FilterDenoise() {    //MLS������С���˷�ƽ���˲�
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("final_datas\\cloud11.pcd", *cloud); 
	
    // Create a KD-Tree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

	// Output has the PointNormal type in order to store the normals calculated by MLS
	pcl::PointCloud<pcl::PointNormal> mls_points;

	// Init object (second point type is for the normals, even if unused)
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

	mls.setComputeNormals(true); //�Ƿ����������

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
