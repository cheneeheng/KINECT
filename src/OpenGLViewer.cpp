/*
 * DataFilter.cpp
 *
 *  Created on: Apr 19, 2017
 *      Author: chen
 */

#include "OpenGLViewer.h"

OpenGLViewer::OpenGLViewer() { }

OpenGLViewer::~OpenGLViewer() { }

void OpenGLViewer::Initialize(
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
		const std::string &obj_)
{
	glGenVertexArrays(1, &VertexArrayID);
	glBindVertexArray(VertexArrayID);
	
	// [OBJ] *******************************************************************
	bool res = loadOBJ(std::string("../object_models/" + obj_ + ".obj").c_str(), vertices);

	std::vector<glm::vec3> verticesC;
	for (int i=0;i<vertices.size();i++) verticesC.push_back(glm::vec3(0.0f,1.0f,0.0f));

	//FOR THE object
	glGenBuffers(1, &vertexbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3), &vertices[0], GL_STATIC_DRAW);

	//For the Color
	glGenBuffers(1, &colorbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
	glBufferData(GL_ARRAY_BUFFER, verticesC.size() * sizeof(glm::vec3), &verticesC[0], GL_STATIC_DRAW);
	// ******************************************************************* [OBJ]

	// [AXES] ******************************************************************
	static const GLfloat g_color_buffer_data[] = { 
		1.0f,  0.0f,  0.0f,
		1.0f,  0.0f,  0.0f,
		0.0f,  1.0f,  0.0f,
		0.0f,  1.0f,  0.0f,
		0.0f,  0.0f,  1.0f,
		0.0f,  0.0f,  1.0f,
	};
	static const GLfloat g_vertex_buffer_data[] = { 
		0.0f, 0.0f, 0.0f,
		0.2f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f,
		0.0f, 0.2f, 0.0f,
		0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, -0.2f,
	};

	//for axes
	glGenBuffers(1, &vertexbufferAxes);
	glBindBuffer(GL_ARRAY_BUFFER, vertexbufferAxes);
	glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data), g_vertex_buffer_data, GL_STATIC_DRAW);
	
	//color
	glGenBuffers(1, &colorbufferAxes);
	glBindBuffer(GL_ARRAY_BUFFER, colorbufferAxes);
	glBufferData(GL_ARRAY_BUFFER, sizeof(g_color_buffer_data), g_color_buffer_data, GL_STATIC_DRAW);
	// ****************************************************************** [AXES]

	// Create and compile our GLSL program from the shaders
	programID =
			LoadShaders(
					"../common/TransformVertexShader.vertexshader",
					"../common/ColorFragmentShader.fragmentshader" );

	// Get a handle for our "MVP" uniform
	MatrixID = glGetUniformLocation(programID, "MVP");	
	ViewMatrixID = glGetUniformLocation(programID, "V");
	ModelMatrixID = glGetUniformLocation(programID, "M");

	ProjectionMatrix = glm::perspective(45.8f, 4.f / 3.f, 0.1f, 100.0f);

	ViewMatrix = glm::lookAt(
			glm::vec3( 0, 0, 0.5 ), // Camera is here
			glm::vec3( 0, 0, 1.5 ), // and looks here
			glm::vec3( 0, 1, 0 )  // Head is up (set to 0,-1,0 to look upside-down)
		);
}


