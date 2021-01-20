/*
 * https://github.com/glaba/STL-Renderer
 *
 * vector.c - source file for ECE220 picture drawing program wrapper code
 *
 * "Copyright (c) 2018 by Charles H. Zega, and Saransh Sinha."
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written agreement is
 * hereby granted, provided that the above copyright notice and the following
 * two paragraphs appear in all copies of this software.
 * 
 * IN NO EVENT SHALL THE AUTHOR OR THE UNIVERSITY OF ILLINOIS BE LIABLE TO 
 * ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL 
 * DAMAGES ARISING OUT  OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, 
 * EVEN IF THE AUTHOR AND/OR THE UNIVERSITY OF ILLINOIS HAS BEEN ADVISED 
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE AUTHOR AND THE UNIVERSITY OF ILLINOIS SPECIFICALLY DISCLAIM ANY 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE 
 * PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND NEITHER THE AUTHOR NOR
 * THE UNIVERSITY OF ILLINOIS HAS ANY OBLIGATION TO PROVIDE MAINTENANCE, 
 * SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS."
 *
 * Author:      Charles Zega, Saransh Sinha
 * Creation Date:   12 February 2018
 * Filename:        renderer.c
 */

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <cstdlib>
#include <cstdio>
#include <fstream>

#include "renderer.hpp"
#include "vector.hpp"
#include <Eigen/Dense>

#define BUFFER_SIZE 2500000
#define STL_BLOCK_SIZE 50

typedef struct {
	int32_t vertices[3];
} Triangle;

typedef struct {
	Vector vertices[3];
} RawTriangle;

int32_t vertex_list_size = 0;
Vector* vertex_list;

int32_t triangles_size = 0;
Triangle* triangles;

Vector get_normal(Triangle t) {
	Vector u = add_vec(vertex_list[t.vertices[1]], neg_vec(vertex_list[t.vertices[0]]));
	Vector v = add_vec(vertex_list[t.vertices[2]], neg_vec(vertex_list[t.vertices[0]]));
	Vector normal;
	normal.x = u.y * v.z - u.z * v.y;
	normal.y = u.z * v.x - u.x * v.z;
	normal.z = u.x * v.y - u.y * v.x;
	return normalize(normal);
}

/*
 * project_point
 *
 * INPUTS: camera_location: the focal point of the camera
 *         camera_direction: the normalized direction of the camera
 *         camera_right: the normalized vector (in 3D) pointing in the (-1, 0) direction in the camera plane
 *         camera_up: the normalizd vector (in 3D) pointing in the (1, 0) direction in the camera plane
 *         origin: the point corresponding to the origin on the camera plane
 *         point: the point to project onto the camera plane
 * RETURNS: A RawTriangle in camera plane coordinates (x, y) if the projection is possible, otherwise it returns a RawTriangle with a negative color
 */
Vector project_point(Vector camera_location, Vector camera_direction, Vector camera_right, Vector camera_up, Vector origin, Vector point) {
	Vector proj;
	proj.z = 0;

	double t = dot(camera_direction, add_vec(origin, neg_vec(camera_location))) / 
	           dot(camera_direction, add_vec(point, neg_vec(camera_location)));
	// Return an impossible projection if t < 0 (non-zero z, where z should be 0)
	if (t < 0) {
		return (Vector){0, 0, -1};
	}

	Vector intersection = (Vector){camera_location.x + (point.x - camera_location.x) * t,
	                               camera_location.y + (point.y - camera_location.y) * t,
	                               camera_location.z + (point.z - camera_location.z) * t};
	intersection = add_vec(intersection, neg_vec(origin));
	proj.x = dot(camera_right, intersection);
	proj.y = dot(camera_up, intersection);
	return proj;
}

/*
 * dynamic_resize
 *
 * INPUTS: array: the array to resize
 *         array_len: a pointer to a number containing the current length of the array
 *         element_size: the size of each element in the array (returned by sizeof)
 * RETURNS: a new array with the old elements but a larger size
 * SIDE EFFECTS: alters the value pointed to by array_len with the new array length
 *               frees the array passed as a parameter
 */
char* dynamic_resize(void* array, int32_t* array_len, uint32_t element_size) {
	char* old_list = (char*)array;
	int32_t old_array_len = *array_len;
	// Double the size of the list
	*array_len = (*array_len == 0) ? (1) : ((*array_len) * 2);
	// Allocate new memory for the new list
    char* new_list = static_cast<char*>(malloc((*array_len) * element_size));
	// Copy the old information from the old array into the new array
	int32_t i;
	for (i = 0; i < old_array_len * element_size; i++) {
		new_list[i] = old_list[i];
	}
	// Free the old list
	free(old_list);
	// Return new list
	return new_list;
}

/*
 * add_triangle
 *
 * INPUTS: t: the triangle object containing the raw coordinates to insert into the scene
 *         num_triangles, num_vertices: pointers to these two counters that will be updated as needed
 */
void add_triangle(RawTriangle t, int32_t* num_triangles, int32_t* num_vertices) {
	int32_t vertexIndices[3];
	int32_t i;
	for (i = 0; i < 3; i++) {
		if (*num_vertices == vertex_list_size) {
			vertex_list = (Vector*) dynamic_resize(vertex_list, &vertex_list_size, (uint32_t)sizeof(Vector));
		}

		vertex_list[*num_vertices] = t.vertices[i];
		vertexIndices[i] = *num_vertices;
		(*num_vertices)++;
	}

	Triangle new_t;
	for (i = 0; i < 3; i++)
		new_t.vertices[i] = vertexIndices[i];

	if (*num_triangles == triangles_size) {
		triangles = (Triangle*) dynamic_resize(triangles, &triangles_size, (uint32_t)sizeof(Triangle));
	}

	triangles[*num_triangles] = new_t;
	(*num_triangles)++;
}

/*
 * parse_and_insert_STL
 *
 * INPUTS: file: the STL file path
 *         max_radius: the maximum distance from the center that each of the vertices in the object should have
 *         num_triangles, num_vertices: pointers to these counters that will be updated as needed when the object is inserted
 *         color: the color of the object
 * SIDE EFFECTS: adds the triangles from the STL file into the scene, and assumes that this object is the only one in the scene
 *               may crash if the file provided is invalid
 */
void parse_and_insert_STL(char* const filepath, double max_radius, int32_t* num_triangles, int32_t* num_vertices) {
	
    FILE* fp;
    fp = fopen(filepath, "r");

	// Skip STL header
	fseek(fp, 80, SEEK_SET);

	// Get number of triangles
	uint32_t actual_num_triangles;
	fread(&actual_num_triangles, 4, 1, fp);

	int32_t i;
	char buffer[BUFFER_SIZE];
	while (1) {
		int32_t num_elements_read = fread(&buffer, 1, BUFFER_SIZE, fp);
		for (i = 0; i < num_elements_read / STL_BLOCK_SIZE; i++) {
			char* cur_buffer = buffer + i * STL_BLOCK_SIZE;
			RawTriangle t;

			// Skip normal
			cur_buffer += 12;

			// Read in triangles
			int32_t j;
			for (j = 0; j < 3; j++) {
				float x, y, z;
				memcpy(&x, cur_buffer + 0, 4);
				memcpy(&y, cur_buffer + 4, 4);
				memcpy(&z, cur_buffer + 8, 4);
				t.vertices[j].x = (double)x;
				t.vertices[j].y = (double)y;
				t.vertices[j].z = (double)z;
				cur_buffer += 12;
			}

			// Skip attributes
			cur_buffer += 2;

			// Add triangle
			add_triangle(t, num_triangles, num_vertices);
		}

		if (num_elements_read != BUFFER_SIZE)
			break;
	}
    fclose(fp);
}	

/*
 * draw_picture
 *
 * Implements a basic 3D renderer that draws the scene described in the function initialize_scene
 * INPUTS: file: the path to the STL file to render
 *         scale: the maximum radius of any of the object's vertices
 *         camera_location: the location where the camera should be placed
 *         rotation: the amount that the camera should be rotated clockwise from its default orientation
 *         color: the color of the object to draw
 * RETURNS: 1 if any dot is drawn out of bounds
 */
Eigen::MatrixXf renderer(char* const filepath, double scale, Eigen::MatrixXf cam_locations, int N, int height, int width) {
    
	// Initialize variables
	int32_t output = 1;
	int32_t num_triangles = 0;
	int32_t num_vertices = 0;
	triangles = NULL;
	vertex_list = NULL;

    // Insert object into scene
	int delta_vertices = num_vertices;
	parse_and_insert_STL(filepath, scale, &num_triangles, &num_vertices);
	delta_vertices = (num_vertices - delta_vertices);
    
    Eigen::MatrixXf picture_data(N * height, width);
    
    for (int nn = 0 ; nn < N ; nn++) {
    Vector camera_location = {cam_locations(nn,0), cam_locations(nn,1), cam_locations(nn,2)}; // translation step between different Eigen and another local vector type
        
        // Clear image
        int32_t x;
        for (x = 0; x < width; x++) {
            int32_t y;
            for (y = 0; y < height; y++) {
                picture_data(y + nn*height,x) = 0.0;
            }
        }
        
	// Set camera direction to point towards the origin (where the object is)
	Vector camera_direction = normalize(neg_vec(camera_location));

        // Set camera right to be the vector in the xy plane that is perpendicular to camera_direction
        double right_angle = -atan2(camera_direction.x, camera_direction.y);
        Vector camera_right = normalize((Vector){cos(right_angle), sin(right_angle), 0});
        
        Vector camera_up = normalize(cross(camera_right, camera_direction));
        
        const double CAMERA_SCALE = 1 / 500.0;
        const double CAMERA_WIDTH = width * CAMERA_SCALE;
        const double CAMERA_HEIGHT = height * CAMERA_SCALE;
        const Vector LIGHT_DIRECTION = camera_direction;
        
        double z_buffer[width][height];
        for (x = 0; x < width; x++) {
            int32_t y;
            for (y = 0; y < height; y++) {
                z_buffer[x][y] = 100000000.0;
            }
        }
        
        // Create z buffer
        // Loop through all objects
        // 		Calculate color of object given its normal
        //		Calculate the positions of the vertices in the picture
        //		Loop through the pixels in the triangle and check each one with the z buffer
        // 		If the z buffer is good, put the pixel into the image
        int32_t i;
        for (i = 0; i < num_triangles; i++)
        {
            Vector projectedVertices[3];
            int32_t j;
            for (j = 0; j < 3; j++) {
                projectedVertices[j] = project_point(camera_location, camera_direction, camera_right, camera_up,
                                                     add_vec(camera_location, camera_direction), vertex_list[triangles[i].vertices[j]]);
            }
            
            Vector proj_start = projectedVertices[0];
            Vector proj_trace = add_vec(projectedVertices[2], neg_vec(projectedVertices[1]));
            Vector actual_start = vertex_list[triangles[i].vertices[0]];
            Vector actual_trace = add_vec(vertex_list[triangles[i].vertices[2]], neg_vec(vertex_list[triangles[i].vertices[1]]));
            
            // Draw a triangle by going across one edge and drawing lines to the remaining point
            double progress;
            double increment1 = CAMERA_SCALE / magnitude(proj_trace) / 2;
            for (progress = 0; progress <= 1; progress += increment1) {
                Vector proj_end = add_vec(projectedVertices[1], mul_vec(progress, proj_trace));
                Vector actual_end = add_vec(vertex_list[triangles[i].vertices[1]], mul_vec(progress, actual_trace));
                
                Vector proj_delta = add_vec(proj_end, neg_vec(proj_start));
                Vector actual_delta = add_vec(actual_end, neg_vec(actual_start));
                
                // Draw a line from start to end
                double t;
                double increment2 = CAMERA_SCALE / magnitude(proj_delta) / 2;
                for (t = 0; t <= 1; t += increment2) {
                    Vector proj_point = add_vec(proj_start, mul_vec(t, proj_delta));
                    Vector actual_point = add_vec(actual_start, mul_vec(t, actual_delta));
                    
                    int32_t camera_x = (proj_point.x + CAMERA_WIDTH / 2) / CAMERA_SCALE;
                    int32_t camera_y = (-proj_point.y + CAMERA_HEIGHT / 2) / CAMERA_SCALE;
                    
                    if (camera_x >= 0 && camera_x < width && camera_y >= 0 && camera_y < height) {
                        // Check distance and z-buffer
                        double dist = magnitude(add_vec(actual_point, neg_vec(camera_location)));
                        if (dist < z_buffer[camera_x][camera_y]) {
                            picture_data(camera_y + nn * height, camera_x) = 1.0;
                        }
                    }
                }
            }
        }
//        std::cout << "yes\n" << picture_data << std::endl;
    }
    
    return picture_data;
}
