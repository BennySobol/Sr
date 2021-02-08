#include "surfaceReconstruction.h"


#define NumberOfNeighborsConsideredForEvaluation 30
#define PercentageOfPointsToRemove 10

// information from https://doc.cgal.org/latest/Manual/tuto_reconstruction.html
surfaceReconstruction::surfaceReconstruction(std::string inputFile, std::string saveFile)
{
    // reading pointcloud
    pointSet points;

    std::ifstream stream(inputFile, std::ios_base::binary);
    if (!stream)
    {
        std::cerr << "Error: cannot read file " << inputFile << std::endl;
        return;
    }
    stream >> points;
    std::cout << "Read " << points.size() << " points" << std::endl;
    if (points.empty())
    {
        return;
    }

    CGAL::remove_outliers<CGAL::Sequential_tag>(points, NumberOfNeighborsConsideredForEvaluation, points.parameters().threshold_percent(PercentageOfPointsToRemove));
    std::cout << points.number_of_removed_points() << " points are outliers" << std::endl;
    // Applying point set processing algorithm to a CGAL::Point_set_3
    // object does not erase the points from memory but place them in
    // the garbage of the object: memory can be freeed by the user.
    points.collect_garbage();
    // Compute average spacing using neighborhood of 6 points
    double spacing = CGAL::compute_average_spacing<CGAL::Sequential_tag>(points, 6);
    // Simplify using a grid of size 2 * average spacing

    CGAL::grid_simplify_point_set(points, 2. * spacing);
    std::cout << points.number_of_removed_points() << " points removed after simplification" << std::endl;
    points.collect_garbage();
    CGAL::jet_smooth_point_set<CGAL::Sequential_tag>(points, 24);

    CGAL::Scale_space_surface_reconstruction_3<kernel> reconstruct(points.points().begin(), points.points().end());
    // Smooth using 4 iterations of Jet Smoothing
    reconstruct.increase_scale(4, CGAL::Scale_space_reconstruction_3::Jet_smoother<kernel>());
    // Mesh with the Advancing Front mesher with a maximum facet length of 0.5
    reconstruct.reconstruct_surface(CGAL::Scale_space_reconstruction_3::Advancing_front_mesher<kernel>(0.5));
    std::cout << "Surface reconstruction is finished" << std::endl;

    // convert reconstruct to cgal mesh
    Mesh cgalMesh;
    auto faces = CGAL::make_range(reconstruct.facets_begin(), reconstruct.facets_end());
    for (pointSet::Index idx : points)
    {
        cgalMesh.add_vertex(points.point(idx));
    }
    for (const auto& face : faces)
    {
        cgalMesh.add_face(Mesh::Vertex_index(face[0]), Mesh::Vertex_index(face[1]), Mesh::Vertex_index(face[2]));
    }

    std::cout << "Colse Holes" << std::endl;
    colseHoles(cgalMesh);

    // convert to pcl PolygonMesh in order to save as ply file
    pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud(new pcl::PointCloud<pcl::PointXYZ>);


    _polygonMesh.polygons.resize(cgalMesh.faces().size());

    for (const Mesh::Vertex_index vertexIdx : cgalMesh.vertices())
    {
        mesh_cloud->push_back(pcl::PointXYZ(cgalMesh.point(vertexIdx).x(), cgalMesh.point(vertexIdx).y(), cgalMesh.point(vertexIdx).z()));
    }
    int i = 0;
    for (const Mesh::Face_index& faceIdx : cgalMesh.faces())
    {
        CGAL::Vertex_around_face_circulator<Mesh> vcirc(cgalMesh.halfedge(faceIdx), cgalMesh), done(vcirc);
        do
        {
            _polygonMesh.polygons[i].vertices.push_back(*vcirc++);
        } while (vcirc != done);
        i++;
    }
    pcl::toPCLPointCloud2(*mesh_cloud, _polygonMesh.cloud);
}

// information from https://doc.cgal.org/latest/Polygon_mesh_processing/index.html#PMPHoleFilling
bool surfaceReconstruction::isHoleToClose(halfedge_descriptor h, Mesh& mesh)
{
    double maxHoleDiam = 500;
    int maxNumHoleEdges = 20;
	int num_hole_edges = 0;
	CGAL::Bbox_3 hole_bbox;
	for (halfedge_descriptor hc : CGAL::halfedges_around_face(h, mesh))
	{
		hole_bbox += mesh.point(target(hc, mesh)).bbox();
        if (++num_hole_edges > maxNumHoleEdges) return false;
		if (hole_bbox.xmax() - hole_bbox.xmin() > maxHoleDiam) return false;
		if (hole_bbox.ymax() - hole_bbox.ymin() > maxHoleDiam) return false;
		if (hole_bbox.zmax() - hole_bbox.zmin() > maxHoleDiam) return false;
	}
	return true;
}

// Incrementally fill the holes that are no larger than given diameter and with no more than a given number of edges
void surfaceReconstruction::colseHoles(Mesh& mesh)
{
	unsigned int nbHoles = 0;
	std::vector<halfedge_descriptor> border_cycles;
	// collect one halfedge per boundary cycle
	CGAL::Polygon_mesh_processing::extract_boundary_cycles(mesh, std::back_inserter(border_cycles));
	for (halfedge_descriptor h : border_cycles)
	{
        if (!isHoleToClose(h, mesh))
        {
            std::cout << "Skipping the hole" << std::endl;
            continue;
        }

		std::vector<boost::graph_traits<Mesh>::face_descriptor>  patch_facets;
		std::vector<boost::graph_traits<Mesh>::vertex_descriptor> patch_vertices;
		bool success = std::get<0>(CGAL::Polygon_mesh_processing::triangulate_refine_and_fair_hole(mesh, h, std::back_inserter(patch_facets), std::back_inserter(patch_vertices)));
		std::cout << "Constructed patch: facets: " << patch_facets.size() << ", vertices: " << patch_vertices.size() << std::endl;
		nbHoles++;
	}
    std::cout << nbHoles << " holes have been filled" << std::endl;
}


// saves the mesh to ply file
void surfaceReconstruction::saveMesh(std::string filePath)
{
    pcl::io::savePLYFile(filePath, _polygonMesh);
}

pcl::PolygonMesh& surfaceReconstruction::getPolygonMesh()
{
    return _polygonMesh;
}