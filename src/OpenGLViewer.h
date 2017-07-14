/*
 * DataFilter.h
 *
 *  Created on: Apr 19, 2017
 *      Author: chen
 */

#ifndef OPENGLVIEWER_H_
#define OPENGLVIEWER_H_

#include <iostream>
#include <GL/glew.h>
#include <glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/quaternion.hpp>
using namespace glm;
#include <common/graphparser.hpp>
#include <common/location_parser.hpp>
#include <common/objectloader.hpp>
#include <common/shader.hpp>
#include <common/quaternion_utils.hpp>

class OpenGLViewer
{
public:
	OpenGLViewer();
	virtual ~OpenGLViewer();

	virtual void Initialize(
		 	GLuint &VertexArrayID,
			GLuint &vertexbuffer,
			GLuint &colorbuffer,
			GLuint &vertexbufferAxes,
			GLuint &colorbufferAxes,
			GLuint &programID,
			GLuint &MatrixID,
			GLuint &ViewMatrixID,
			GLuint &ModelMatrixID,
			glm::mat4 &ViewMatrix,
			glm::mat4 &ProjectionMatrix,
			std::vector<glm::vec3> &vertices,
			const std::string &obj_);
};

#endif /* OPENGLVIEWER_H_ */
