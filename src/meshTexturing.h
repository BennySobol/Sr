#pragma once

#include <fstream>
#include <sstream>
#include "features.h"

#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/io/obj_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/transforms.h>
#include <pcl/common/transforms.h>
#include <pcl/surface/texture_mapping.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

class meshTexturing
{
private:
	// The texture mesh object that will contain our UV-mapped mesh
	pcl::TextureMesh _texturedMesh;
public:
	meshTexturing(std::vector<imageFeatures>& features, cameraCalibration& calib, std::string plyFilePath, std::string saveObjFilePath, pcl::PolygonMesh triangles);
	void saveTextureMesh(std::string filePath);
};

