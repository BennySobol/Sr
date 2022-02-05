#pragma once
#include <iostream>
#include <vector>
#include <fstream>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include <pcl/io/ply_io.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Point_set_3/IO.h>
#include <CGAL/remove_outliers.h>
#include <CGAL/grid_simplify_point_set.h>
#include <CGAL/jet_smooth_point_set.h>
#include <CGAL/jet_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/poisson_surface_reconstruction.h>
#include <CGAL/Advancing_front_surface_reconstruction.h>
#include <CGAL/Scale_space_surface_reconstruction_3.h>
#include <CGAL/Scale_space_reconstruction_3/Jet_smoother.h>
#include <CGAL/Scale_space_reconstruction_3/Advancing_front_mesher.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/IO/write_ply_points.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/Polygon_mesh_processing/border.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel kernel;
typedef CGAL::Point_set_3<kernel::Point_3, kernel::Vector_3> pointSet;

typedef CGAL::Surface_mesh<kernel::Point_3> Mesh;
typedef boost::graph_traits<Mesh>::halfedge_descriptor halfedge_descriptor;


class surfaceReconstruction
{
private:
	pcl::PolygonMesh _polygonMesh;

	void colseHoles(Mesh& mesh);
	bool isHoleToClose(halfedge_descriptor h, Mesh& mesh);

public:
	surfaceReconstruction(std::string inputFile, std::string saveFile);
	~surfaceReconstruction();

	void saveMesh(std::string filePath);
	pcl::PolygonMesh& getPolygonMesh();
};

