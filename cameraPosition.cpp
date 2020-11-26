#include "cameraPosition.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;
using namespace Eigen;


struct CloudPoint {
	cv::Point3d pt;
	std::vector<int>index_of_2d_origin;
	float color;
};



void convertHomogeneous(cv::Mat point4D, std::vector< cv::Point3d>& point3D);

void obtainMatches(vector<KeyPoint>& kp1, vector<KeyPoint>& kp2, Mat& descriptors1, Mat& descriptors2, vector<Point2d>& points1, vector<Point2d>& points2, vector<int>& points1_idx, vector<int>& points2_idx);


cameraPosition::cameraPosition(cameraCalibration calib, std::vector<imageFeatures> features)
{
	matchingKeyPoints matchingKeyPoints = features[0].matchingKeyPoints;

	cv::Mat E, rotation, translation, mask;
	E = findEssentialMat(matchingKeyPoints.currentKeyPoints, matchingKeyPoints.otherKeyPoints, calib.getFocal(), calib.getPP(), cv::RANSAC, 0.9, 3.0, mask);
	cv::recoverPose(E, matchingKeyPoints.currentKeyPoints, matchingKeyPoints.otherKeyPoints, rotation, translation, calib.getFocal(), calib.getPP(), mask);

	std::vector<cv::Mat> v_transf;

	cv::Mat zeros = (cv::Mat_<double>(3, 1) << 0, 0, 0);

	std::vector<CloudPoint> todos_los_puntos_en_3D;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

	//
	cv::Mat tran, tran1, points4D, A, B;

	hconcat(rotation, translation, tran);
	hconcat(calib.getCameraMatrix(), zeros, tran1);
	A = tran1;
	B = calib.getCameraMatrix() * tran;
	v_transf.push_back(B);

	triangulatePoints(A, B, matchingKeyPoints.currentKeyPoints, matchingKeyPoints.otherKeyPoints, points4D);
	std::vector<cv::Point3d> PointCloud;
	convertHomogeneous(points4D, PointCloud);

	// 
	for (int i = 0; i < points4D.cols; i++) {
		CloudPoint point;
		pcl::PointXYZRGB point_pcl;

		point.pt = PointCloud[i];
		point_pcl.x = point.pt.x;
		point_pcl.y = point.pt.y;
		point_pcl.z = point.pt.z;

		std::vector<int> indices(features.size(), 0);
		indices[0] = matchingKeyPoints.currentKeyPointsIdx[i];
		indices[1] = matchingKeyPoints.otherKeyPointsIdx[i];
		point.index_of_2d_origin = indices;

		cv::Vec3b color = features[0].image.at<cv::Vec3b>(cv::Point(matchingKeyPoints.currentKeyPoints[i].x, matchingKeyPoints.currentKeyPoints[i].y));
		uint8_t r = (uint8_t)color.val[2];
		uint8_t g = (uint8_t)color.val[1];
		uint8_t b = (uint8_t)color.val[0];
		uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
		point_pcl.rgb = *reinterpret_cast<float*>(&rgb);
		point.color = *reinterpret_cast<float*>(&rgb);

		todos_los_puntos_en_3D.push_back(point);
		point_cloud_ptr->push_back(point_pcl);
	}

	////myfile << "points1 = [";
	////for (int k = 0; k < (int)todos_los_puntos_en_3D.size(); k++) {
	////    myfile << todos_los_puntos_en_3D[k].pt.x << ", " << todos_los_puntos_en_3D[k].pt.y << ", " << todos_los_puntos_en_3D[k].pt.z << endl;
	////}
	////myfile << "]" << endl;

	 /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	  * 3
	  * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	std::vector<CloudPoint> puntos_en_3D_anteriores = todos_los_puntos_en_3D;


	for (int i = 2; i < features.size() - 1; i++) {

		std::vector<cv::KeyPoint> keypoint3d;
		cv::KeyPoint key3d;
		cv::Mat descriptors3d;
		int posicion;

		for (unsigned int j = 0; j < puntos_en_3D_anteriores.size(); j++) {
			//			cout << i << ", " << j << endl;
			posicion = puntos_en_3D_anteriores[j].index_of_2d_origin[i - 1];
			//			cout << "j: " << j << " --  i-1: " << i-1 << " -- posicion: " << posicion << " -- v_desc_tot: " << v_descriptors[i-1].size() << endl;
			descriptors3d.push_back(features[i - 1].descriptors.row(posicion));// -1?
			//			cout << "C" << endl;
			key3d = features[i - 1].keyPoints[posicion]; // -1?
			keypoint3d.push_back(key3d);
		}
		std::cout << "3d" << keypoint3d.size() << ", " << "kp" << features[i].keyPoints.size() << std::endl;


		std::vector<cv::Point2d> obj_new, scene_new;
		std::vector<int> obj_idx_new, scene_idx_new;
		obtainMatches(keypoint3d, features[i].keyPoints, descriptors3d, features[i].descriptors, obj_new, scene_new, obj_idx_new, scene_idx_new);

		cv::Mat rvec, tvec;
		std::vector<cv::Point3d> pnpPointcloud_valid;
		cv::Point3d  punto;

		for (unsigned int j = 0; j < obj_idx_new.size(); j++) {
			punto = puntos_en_3D_anteriores[obj_idx_new[j]].pt;
			pnpPointcloud_valid.push_back(punto);
		}
		std::cout << pnpPointcloud_valid.size() << ", " << scene_new.size() << std::endl;
		cv::solvePnPRansac(pnpPointcloud_valid, scene_new, calib.getCameraMatrix(), calib.getDistortionCoefficients(), rvec, tvec, false, 300, 3.0); //19 error OpenCV(4.4.0-dev) Error: Unspecified error (> DLT algorithm needs at least 6 points for pose estimation from 3D-2D point correspondences. (expected: 'count >= 6'), where 'count' is 5 must be greater than or equal to '6' is 6) in void __cdecl cvFindExtrinsicCameraParams2(const struct CvMat*, const struct CvMat*, const struct CvMat*, const struct CvMat*, struct CvMat*, struct CvMat*, int), file C : \cv\opencv - master\modules\calib3d\src\calibration.cpp, line 1173

		std::cout << "Ransac done!" << endl;

		// Convert from Rodrigues format to rotation matrix and build the 3x4 Transformation matrix
		cv::Mat r_mat;
		cv::Rodrigues(rvec, r_mat);
		hconcat(r_mat, tvec, tran);

		// We triangulate the points
		B.release();
		B = calib.getCameraMatrix() * tran;
		v_transf.push_back(B);

		cv::Mat new_points4D;
		std::vector<cv::Point3d> new_PointCloud;
		cout << "Imagen " << i - 1 << " con imagen " << i << ": " << features[i - 1].matchingKeyPoints.currentKeyPoints.size() << endl;
		triangulatePoints(v_transf[i - 2], v_transf[i - 1], features[i - 1].matchingKeyPoints.currentKeyPoints, features[i - 1].matchingKeyPoints.otherKeyPoints, new_points4D);
		//		cout << new_points4D << endl;
		convertHomogeneous(new_points4D, new_PointCloud);

		puntos_en_3D_anteriores.clear();

		for (int j = 0; j < (int)new_PointCloud.size(); j++) {

			if ((features[i - 1].matchingKeyPoints.currentKeyPointsIdx[j] < (int)features[i - 1].keyPoints.size())
				&& (features[i - 1].matchingKeyPoints.currentKeyPointsIdx[j] >= 0)
				&& (features[i - 1].matchingKeyPoints.otherKeyPointsIdx[j] < (int)features[i].keyPoints.size())
				&& (features[i - 1].matchingKeyPoints.otherKeyPointsIdx[j] >= 0)) {

				CloudPoint point;
				pcl::PointXYZRGB point_pcl;

				point.pt = new_PointCloud[j];
				point_pcl.x = point.pt.x;
				point_pcl.y = point.pt.y;
				point_pcl.z = point.pt.z;

				// You only have a couple of images so the vector has to correspond only to these two images
				std::vector<int> indices(features.size(), 0);

				indices[i - 1] = features[i - 1].matchingKeyPoints.currentKeyPointsIdx[j];
				indices[i] = features[i - 1].matchingKeyPoints.otherKeyPointsIdx[j];
				// cout << point.pt <<  " [" << indices[0] << ", " << indices[1] << ", " << indices[2] << ", " << indices[3] << ", " << indices[4] << "]" << endl;
				point.index_of_2d_origin = indices;

				cv::Vec3b color = features[i - 1].image.at<cv::Vec3b>(cv::Point(features[i - 1].matchingKeyPoints.currentKeyPoints[i].x, features[i - 1].matchingKeyPoints.currentKeyPoints[i].y)); //???????
				uint8_t r = (uint8_t)color.val[2];
				uint8_t g = (uint8_t)color.val[1];
				uint8_t b = (uint8_t)color.val[0];
				uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
				point.color = *reinterpret_cast<float*>(&rgb);
				point_pcl.rgb = *reinterpret_cast<float*>(&rgb);

				todos_los_puntos_en_3D.push_back(point);
				puntos_en_3D_anteriores.push_back(point);
				point_cloud_ptr->push_back(point_pcl);

			}
		}
		////myfile << "points" << i << " = [";
		////for (int k = 0; k < (int)puntos_en_3D_anteriores.size(); k++) {
		////    myfile << puntos_en_3D_anteriores[k].pt.x << ", " << puntos_en_3D_anteriores[k].pt.y << ", " << puntos_en_3D_anteriores[k].pt.z << endl;
		////}
		////myfile << "]" << endl;
	}

	// pcl showCloud
	pcl::visualization::CloudViewer viewer("Viewer");
	viewer.showCloud(point_cloud_ptr);
	while (!viewer.wasStopped()) {

	}
	cv::destroyAllWindows();
}



void convertHomogeneous(cv::Mat point4D, std::vector< cv::Point3d>& point3D) {
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> change;

	cv2eigen(point4D, change);

	for (int b = 0; b < point4D.cols; b++) {
		cv::Point3d mA;
		if (change(3, b) == 0) {
			mA.x = change(0, b);
			mA.y = change(1, b);
			mA.z = change(2, b);
		}
		else {
			mA.x = change(0, b) / change(3, b);
			mA.y = change(1, b) / change(3, b);
			mA.z = change(2, b) / change(3, b);
		}
		point3D.push_back(mA);
	}
}



void obtainMatches(vector<KeyPoint>& kp1, vector<KeyPoint>& kp2, Mat& descriptors1, Mat& descriptors2, vector<Point2d>& points1, vector<Point2d>& points2, vector<int>& points1_idx, vector<int>& points2_idx) {

	BFMatcher matcher(NORM_L2, false);
	vector<vector< DMatch > > matches;
	matcher.knnMatch(descriptors1, descriptors2, matches, 2);

	vector<DMatch > Best_Matches;
	for (int k = 0; k < (int)matches.size(); k++) {

		float dis1 = matches[k][0].distance;
		float dis2 = matches[k][1].distance;

		if (((dis1 < 300.0 && dis1 > 0) || (dis2 < 300.0 && dis2 > 0)) && (dis2 / dis1 > 1.5))
			Best_Matches.push_back(matches[k][0]);

		for (int l = 0; l < (int)Best_Matches.size(); l++) {
			points1.push_back(kp1[Best_Matches[l].queryIdx].pt);
			points1_idx.push_back(Best_Matches[l].queryIdx);

			points2.push_back(kp2[Best_Matches[l].trainIdx].pt);
			points2_idx.push_back(Best_Matches[l].trainIdx);
		}
	}
}


////// Recover motion between first to second image
////cv::Mat Transformation = cv::Mat::eye(4, 4, CV_64F);
////cv::Mat Projection = calib.getCameraMatrix() * cv::Mat::eye(3, 4, CV_64F);

////// pose from next to current 

////recoverPose(E, point2, point1, local_R, local_t, K.at<double>(0, 0), cv::Point2d(K.at<double>(0, 2), K.at<double>(1, 2)), mask);

////// local tansform
////cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
////local_R.copyTo(T(cv::Range(0, 3), cv::Range(0, 3)));
////local_t.copyTo(T(cv::Range(0, 3), cv::Range(3, 4)));

////// accumulate transform
////cv::Mat nextTransformation = Transformation * T;


////// make projection matrix
////cv::Mat R = nextTransformation(cv::Range(0, 3), cv::Range(0, 3));
////cv::Mat t = nextTransformation(cv::Range(0, 3), cv::Range(3, 4));

////cv::Mat P(3, 4, CV_64F);

////P(cv::Range(0, 3), cv::Range(0, 3)) = R.t();
////P(cv::Range(0, 3), cv::Range(3, 4)) = -R.t() * t;
////P = K * P;

////cv::Mat nextProjection = P;

////cv::Mat points4D;
////triangulatePoints(Projection, nextProjection, point1, point2, points4D);

////// display point cloud

////// prepare a viewer
////pcl::visualization::PCLVisualizer viewer("Viewer");
////viewer.setBackgroundColor(255, 255, 255);

////// create point cloud
////pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
////cloud->points.resize(points4D.rows);

////for (int i = 0; i < points4D.rows; i++) {
////    pcl::PointXYZRGB& point = cloud->points[i];

////    point.x = points4D.at<float>(0, i);
////    point.y = points4D.at<float>(1, i);
////    point.z = points4D.at<float>(2, i);
////    point.r = 0;
////    point.g = 0;
////    point.b = 255;
////}

////viewer.addPointCloud(cloud, "Point Cloud");
////viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Point Cloud"); viewer.addCoordinateSystem(1.0);

////viewer.initCameraParameters();
////while (!viewer.wasStopped()) {
////    viewer.spin();
////}