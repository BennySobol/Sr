#include "meshTexturing.h"
#include "loadImages.h"

// mesh texturing inspiration from main function at https://github.com/PointCloudLibrary/pcl/blob/master/gpu/kinfu_large_scale/tools/standalone_texture_mapping.cpp
meshTexturing::meshTexturing(std::string plyFilePath, std::string saveObjFilePath, pcl::PolygonMesh& triangles)
{
	std::vector<imageFeatures>& features = features::getFeatures();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(triangles.cloud, *cloud);

	std::cout << "Estimating normals" << std::endl;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);

	// concatenate point cloud and normal fields
	pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloudWithNormals);


	_texturedMesh.cloud = triangles.cloud;
	std::vector<pcl::Vertices> polygon;

	for (size_t i = 0; i < triangles.polygons.size(); ++i)
	{
		polygon.push_back(triangles.polygons[i]);
	}
	_texturedMesh.tex_polygons.push_back(polygon);
	std::cout << "Mesh contains " << _texturedMesh.tex_polygons[0].size() << " faces" << std::endl;

	pcl::texture_mapping::CameraVector cameras;
	for (imageFeatures feature : features)
	{
		pcl::TextureMapping<pcl::PointXYZ>::Camera cam;
		cam.pose = feature.getCamPose();
		cv::Mat cameraMatrix = cameraCalibration::getCameraMatrix();
		cam.focal_length_w = cameraMatrix.at<double>(0, 0);
		cam.focal_length_h = cameraMatrix.at<double>(1, 1);
		cv::Point2d principlePoint = cameraCalibration::getPrinciplePoint();
		cam.center_w = principlePoint.x;
		cam.center_h = principlePoint.y;
		cam.height = feature.image.size().height;
		cam.width = feature.image.size().width;
		cam.texture_file = feature.path;

		cameras.push_back(cam);
	}
	// create materials for each texture and one extra for occluded faces
	for (int i = 0; i <= cameras.size(); i++)
	{
		pcl::TexMaterial meshMaterial;
		meshMaterial.tex_Ka.r = 0.2f;
		meshMaterial.tex_Ka.g = 0.2f;
		meshMaterial.tex_Ka.b = 0.2f;

		meshMaterial.tex_Kd.r = 0.8f;
		meshMaterial.tex_Kd.g = 0.8f;
		meshMaterial.tex_Kd.b = 0.8f;

		meshMaterial.tex_Ks.r = 1.0f;
		meshMaterial.tex_Ks.g = 1.0f;
		meshMaterial.tex_Ks.b = 1.0f;

		meshMaterial.tex_d = 1.0f;
		meshMaterial.tex_Ns = 75.0f;
		meshMaterial.tex_illum = 2;

		std::stringstream texName;
		texName << "material_" << i;
		texName >> meshMaterial.tex_name;

		if (i < cameras.size())
		{
			meshMaterial.tex_file = "../" + getFileNameWithExtension(cameras[i].texture_file);
		}
		else
		{
			meshMaterial.tex_file = "occluded.jpg";
		}

		_texturedMesh.tex_materials.push_back(meshMaterial);
	}

	// segment and texture
	std::cout << "Finding occlusions" << std::endl;
	pcl::TextureMapping<pcl::PointXYZ> tm;
	tm.textureMeshwithMultipleCameras(_texturedMesh, cameras);

	for (int i = 0; i < cameras.size(); i++)
	{
		std::cout << "Camera " << getFileName(features[i].path) << " is texturing " << _texturedMesh.tex_polygons[i].size() << " faces" << std::endl;
	}

	// converting cloud with normals to ROS format
	pcl::toPCLPointCloud2(*cloudWithNormals, _texturedMesh.cloud);

	saveTextureMesh(saveObjFilePath);
	std::cout << "Textured mesh has been saved" << std::endl;
}


// saves the texture mesh to file
void meshTexturing::saveTextureMesh(std::string filePath)
{
	pcl::io::saveOBJFile(filePath, _texturedMesh);
}

meshTexturing::~meshTexturing()
{
	_texturedMesh.cloud.data.clear();
	_texturedMesh.tex_coordinates.clear();
	_texturedMesh.tex_materials.clear();
	_texturedMesh.tex_polygons.clear();
}
