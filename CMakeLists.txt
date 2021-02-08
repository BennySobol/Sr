cmake_minimum_required(VERSION 3.10)
# Create Project
project( SFM_Project )
add_executable( project main.cpp loadImages.cpp loadImages.h features.cpp features.h cameraCalibration.cpp cameraCalibration.h structureFromMotion.cpp structureFromMotion.h bundleAdjustment.cpp bundleAdjustment.h surfaceReconstruction.cpp surfaceReconstruction.h meshHolesFiller.cpp meshHolesFiller.h )

set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "project" )

# Find PCL
find_package( PCL 1.11.1 REQUIRED )

# Find OpenCV
set( OpenCV_DIR "C:/opencv/build" )
find_package( OpenCV REQUIRED )

set( Eigen3_DIR "C:/ceres/eigen-build" )

# Find CERES
find_package( Ceres REQUIRED )

# Find CGAL
find_package( CGAL REQUIRED )

# [Linker]>[Input]>[Additional Dependencies]
target_link_libraries( project ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${CERES_LIBRARIES} ${CGAL_LIBRARIES} )


