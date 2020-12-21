cmake_minimum_required( VERSION 2.8 )
# Create Project
project( SFM_Project )
add_executable( project main.cpp loadImages.cpp loadImages.h features.cpp features.h cameraCalibration.cpp cameraCalibration.h cameraPosition.cpp cameraPosition.h bundleAdjustment.cpp bundleAdjustment.h)

# Set StartUp Project (Option)
# (This setting is able to enable by using CMake 3.6.0 RC1 or later.)
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "project" )

# Find Packages
# Find PCL
find_package( PCL 1.11.1 REQUIRED )

# Find OpenCV
set( OpenCV_DIR "C:/opencv/build" )
find_package( OpenCV REQUIRED )

find_package(Ceres)


if( PCL_FOUND AND OpenCV_FOUND AND Ceres_FOUND)
  # [C/C++]>[General]>[Additional Include Directories]
  include_directories( ${PCL_INCLUDE_DIRS} )
  include_directories( ${OpenCV_INCLUDE_DIRS} )
  include_directories( ${CERES_INCLUDE_DIRS} )

  # [C/C++]>[Preprocessor]>[Preprocessor Definitions]
  add_definitions( ${PCL_DEFINITIONS} )

  # For Use Not PreCompiled Features 
  #add_definitions( -DPCL_NO_PRECOMPILE )

  # [Linker]>[General]>[Additional Library Directories]
  link_directories( ${PCL_LIBRARY_DIRS} )
  link_directories( ${OpenCV_LIB_DIR} )

  
  # [Linker]>[Input]>[Additional Dependencies]
  target_link_libraries( project ${PCL_LIBRARIES} )
  target_link_libraries( project ${OpenCV_LIBS} )
  target_link_libraries( project ${CERES_LIBRARIES})
endif()