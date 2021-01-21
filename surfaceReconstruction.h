#pragma once
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

#include <iostream>
#include <vector>
#include <fstream>


typedef CGAL::Exact_predicates_inexact_constructions_kernel kernel;
typedef CGAL::Point_set_3<kernel::Point_3, kernel::Vector_3> pointSet;


class surfaceReconstruction
{
private:

public:
    surfaceReconstruction(std::string inputFile, std::string saveFile, unsigned int reconstructionChoice=1);
    ~surfaceReconstruction() = default;
};

