cmake_minimum_required(VERSION 3.10)
# Create Project
project( Sr_Project )

file(GLOB SRC_FILES "src/*.h" "src/*.cpp")
add_executable(project ${SRC_FILES})

set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "project" )

# Find PCL
find_package( PCL REQUIRED )

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

# install(TARGETS project DESTINATION bin)

