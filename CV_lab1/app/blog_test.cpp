#include <iostream>
#include <fstream>
#include <string>
using namespace std;

#include <opencv2/opencv.hpp>

#include <pcl-1.10/pcl/io/pcd_io.h>
#include <pcl-1.10/pcl/point_types.h>
#include <pcl-1.10/pcl/visualization/cloud_viewer.h>


cv::Mat_<double> LinearLSTriangulation(
		cv::Point3d u,	//homogenous image point (u,v,1)  
		cv::Matx34d P,	//camera 1 matrix  
		cv::Point3d u1,	//homogenous image point in 2nd camera  
		cv::Matx34d P1	//camera 2 matrix  
);

bool CheckCoherentRotation(cv::Mat_<double>& R);

//输出文件函数：
string getfilename(string filename1,int i,string filename2);

cv::FileStorage parameter_file;
void set_parameter_file(const string& file_name)
{
    parameter_file = cv::FileStorage(file_name.c_str(), cv::FileStorage::READ);
    if (!parameter_file.isOpened())
    {
        cerr << "parameter file not found.\n";
        exit(1);
    }
}

template<typename T>
T get_parameter(const string& key)
{
    return T(parameter_file[key]);
}

int main(int argc, char** argv)
{	
	if (argc != 2)
	{
		cerr << "usage: blog_test <config_file path>";
		exit(1);
	}
	set_parameter_file(string(argv[1]));
	//点云显示框建立
	pcl::visualization::PCLVisualizer viewer;
	cout << "正在生成点云，请等待..." << endl;
	
	//相机标定部分

	ifstream fin; 	// 标定所用图像文件的路径 
	ofstream fout;  // 保存标定结果的文件 
	int k  = 1;
	int ii = 0;

	cv::Matx33d K(
		get_parameter<double>("camera.fx"), 0.0, get_parameter<double>("camera.cx"),
		0.0, get_parameter<double>("camera.fy"), get_parameter<double>("camera.cy"),
		0.0, 0.0, 1.0
	);
	cout << "K=" << K << endl;
	//sift匹配部分
	fin.open(get_parameter<string>("image_list"));
	if (!fin.is_open())
	{
		cerr << "parameter image_list not found...\n";
		exit(2);
	}
	string filename1, filename2;

	while (getline(fin, filename1))
	{
		if (filename1 == "")
			break;
		while (getline(fin, filename2))
		{
			//读入图片
			if (filename2 == "")
				break;
			cv::Mat img_1 = cv::imread(filename1);
			cv::Mat img_2 = cv::imread(filename2);
			if (img_1.data == nullptr || img_2.data == nullptr)
			{
				cerr << "image not exist...\n";
				exit(2);
			}
			cout << "输入图片为：" << filename1 << endl;
			cout << "输入图片为：" << filename2 << endl;
			
			//Create SIFT class pointer
			cv::Ptr<cv::Feature2D> f2d = cv::ORB::create();
			//Detect the keypoints
			vector<cv::KeyPoint> keypoints_1, keypoints_2;
			f2d->detect(img_1, keypoints_1);
			f2d->detect(img_2, keypoints_2);
			//Calculate descriptors (feature vectors)
			cv::Mat descriptors_1, descriptors_2;
			f2d->compute(img_1, keypoints_1, descriptors_1);
			f2d->compute(img_2, keypoints_2, descriptors_2);
			//Matching descriptor vector using BFMatcher
			cv::BFMatcher matcher;
			vector<cv::DMatch> matches;
			matcher.match(descriptors_1, descriptors_2, matches);
			//此处应该有掌声，谢谢大佬
			int ptCount = (int)matches.size();
			cout << "第" << k << "次匹配点云数量为：" << ptCount << endl;
			cv::Mat p1(ptCount, 2, CV_32F);
			cv::Mat p2(ptCount, 2, CV_32F);
			// 把Keypoint转换为Mat
			//cout << "关键点数目：" << ptCount << endl;
			cv::Point2f pt;
			for (int i = 0; i < ptCount; i++)
			{
				pt = keypoints_1[matches[i].queryIdx].pt;
				p1.at<float>(i, 0) = pt.x;
				p1.at<float>(i, 1) = pt.y;

				pt = keypoints_2[matches[i].trainIdx].pt;
				p2.at<float>(i, 0) = pt.x;
				p2.at<float>(i, 1) = pt.y;
			}
			cv::Mat F;
			F = findFundamentalMat(p1, p2, cv::FM_RANSAC);
			cout << "基础矩阵为：" << F << endl;
			//绘制匹配出的关键点
			cv::Mat img_matches;
			drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_matches);

			//重建部分
			cv::Mat_<double> E = cv::Mat(K.t()) * F * cv::Mat(K);
			cv::SVD svd(E);
			cv::Matx33d W(0, -1, 0,//HZ 9.13  
				1, 0, 0,
				0, 0, 1);
			cv::Mat_<double> R = svd.u * cv::Mat(W) * svd.vt; //HZ 9.19  
			cv::Mat_<double> t = svd.u.col(2); //u3 
			if (!CheckCoherentRotation(R))
				cout << "resulting rotation is not coherent\n";

			cv::Matx34d P(
				1, 0, 0, 0,
				0, 1, 0, 0,
				0, 0, 1, 0
			);
			cv::Matx34d P1(
				R(0, 0), R(0, 1), R(0, 2), t(0),
				R(1, 0), R(1, 1), R(1, 2), t(1),
				R(2, 0), R(2, 1), R(2, 2), t(2)
			);

			cv::Mat p_1(ptCount, 2, CV_32F);
			cv::Mat p_2(ptCount, 2, CV_32F);
			// 把Keypoint转换为Mat
			for (int i = 0; i < ptCount; i++)
			{
				pt = keypoints_1[matches[i].queryIdx].pt;
				p_1.at<float>(i, 0) = pt.x;
				p_1.at<float>(i, 1) = pt.y;

				pt = keypoints_2[matches[i].trainIdx].pt;
				p_2.at<float>(i, 0) = pt.x;
				p_2.at<float>(i, 1) = pt.y;
			}


			cv::Mat Kinv;
			invert(K, Kinv, cv::DECOMP_LU);
			cout << "这个矩阵为："<<Kinv << endl;
			//vector<Point3d> u_1, u_2;
			vector<cv::Point3d> pointcloud;
			cv::Mat_<double> X_1, X;
			vector<double> reproj_error;
			for (size_t i = 0; i < ptCount; i++)
			{
				cv::Point2f kp  = keypoints_1[matches[i].queryIdx].pt;
				cv::Point2f kp1 = keypoints_2[matches[i].trainIdx].pt;
				
				cv::Point3d	u_1(kp.x, kp.y, 1);
				cv::Point3d	u_2(kp1.x, kp1.y, 1);
				//cout << "abc"<<u_1 << u_2<<endl;
				cv::Mat_<double> um_1 = Kinv * cv::Mat_<double>(u_1);
				//	cout <<"ijk"<< um_1 << endl;
				u_1 = cv::Point3d(um_1);
				cv::Mat_<double> um_2 = Kinv * cv::Mat_<double>(u_2);
				u_2 = cv::Point3d(um_2);
				//	cout << "xyz" << u_1 << u_2 << endl;
				X_1 = LinearLSTriangulation(u_1, P, u_2, P1);
				X.push_back(X_1);
				pointcloud.push_back(cv::Point3d(X_1(0), X_1(1), X_1(2)));
			}

			string filename3 = getfilename("pointcloud", k, ".txt");
			fout.open(filename3);
			fout << "点云结果为：" << endl;
			fout << pointcloud;
			fout.close();

			pcl::PointCloud<pcl::PointXYZ> pointcloud1;
			pointcloud1.width = ptCount;
			pointcloud1.height = 1;
			pointcloud1.is_dense = false;
			pointcloud1.points.resize(pointcloud1.width * pointcloud1.height);
			for (size_t i = 0; i < ptCount; i++)
			{
				pointcloud1[i].x = pointcloud[i].x;
				pointcloud1[i].y = pointcloud[i].y;
				pointcloud1[i].z = pointcloud[i].z;
			}
			string filename4 = getfilename("pointcloud", k, ".pdb");
			pcl::io::savePCDFileASCII(filename4, pointcloud1);
			string filename5;
			filename5 = getfilename("cloud", k, "");
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>); // 创建点云（指针） 
			//string filename6 = getfilename("pointcloud", k, "pdb");
			pcl::io::loadPCDFile(filename4, *cloud1);
			viewer.addPointCloud(cloud1, filename5);
			k++;
		} // end of inner while loop
		
		ii++;
		fin.close();
		fin.open("sift.txt");
		for (int j = 0; j < ii; j++)
		{
			getline(fin, filename1);
		}
	} // end of outer while loop


	cout << "点云结果如图所示" << endl;
	viewer.spin();
	system("pause");

	return 0;
}

//函数定义
bool CheckCoherentRotation(cv::Mat_<double>& R) 
{
	if (fabsf(determinant(R)) - 1.0 > 1e-07) {
		cerr << "det(R) != +-1.0, this is not a rotation matrix" << endl;
		return false;
	}
	return true;
}

cv::Mat_<double> LinearLSTriangulation(
		cv::Point3d u,	//homogenous image point (u,v,1)  
		cv::Matx34d P,	//camera 1 matrix  
		cv::Point3d u1,	//homogenous image point in 2nd camera  
		cv::Matx34d P1	//camera 2 matrix  
)
{
	//build A matrix  
	cv::Matx43d A(u.x*P(2, 0) - P(0, 0), u.x*P(2, 1) - P(0, 1), u.x*P(2, 2) - P(0, 2),
		u.y*P(2, 0) - P(1, 0), u.y*P(2, 1) - P(1, 1), u.y*P(2, 2) - P(1, 2),
		u1.x*P1(2, 0) - P1(0, 0), u1.x*P1(2, 1) - P1(0, 1), u1.x*P1(2, 2) - P1(0, 2),
		u1.y*P1(2, 0) - P1(1, 0), u1.y*P1(2, 1) - P1(1, 1), u1.y*P1(2, 2) - P1(1, 2)
		);
	//build B vector  
	cv::Matx41d B(-(u.x*P(2, 3) - P(0, 3)),
		-(u.y*P(2, 3) - P(1, 3)),
		-(u1.x*P1(2, 3) - P1(0, 3)),
		-(u1.y*P1(2, 3) - P1(1, 3)));
	//solve for X  
	cv::Mat_<double> X;
	cv::solve(A, B, X, cv::DECOMP_SVD);
	return X;
}

string getfilename(string filename1, int i, string filename2)
{
	//第i个文件名为：
	string filename = filename1 + to_string(i) + filename2;
	return filename;
}