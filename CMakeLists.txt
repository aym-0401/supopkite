cmake_minimum_required(VERSION 3.5)

project(supopkite LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

set(	SOURCES
	src/main.cpp

	src/tracking.cpp
	src/orientation_4pts.cpp
	src/generate_database.cpp

	src/camera_calibration.cpp

	src/renderer.cpp
	src/point_projection.cpp
	src/homography.cpp
	src/vector.cpp

#todo	src/write_data.cpp

	ext/kissfft/src/kiss_fft.c
	ext/kissfft/src/kiss_fftnd.c
	ext/kissfft/src/kiss_fftndr.c
	ext/kissfft/src/kiss_fftr.c
	)

set(	HEADERS

	src/tracking.hpp
	src/orientation_4pts.hpp
	src/generate_database.hpp

	src/renderer.hpp
	src/point_projection.hpp
	src/homography.hpp
	src/vector.hpp

	ext/kissfft/include/_kiss_fft_guts.h
	ext/kissfft/include/kiss_fft.h
	ext/kissfft/include/kiss_fft_log.h
	ext/kissfft/include/kiss_fftnd.h
	ext/kissfft/include/kiss_fftndr.h
	ext/kissfft/include/kiss_fftr.h
	)

add_executable(supopkite ${SOURCES} ${HEADERS})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${EIGEN3_LIBS})