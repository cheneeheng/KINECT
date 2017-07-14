/*
 * main.cpp
 *
 *  Created on: Dec 6, 2010
 *      Author: papazov
 */


#include <Eigen/Core>
#include <Eigen/Dense>
#include <fstream>
#include <memory>
#include <iostream>
#include <pthread.h>
#include <vector>
#include <semaphore.h>
#include <sstream>
#include <string>

#include <opencv2/opencv.hpp>

//#include "dataDeclaration.h"
#include "PFTracker.h"
#include "ObjectDetection.h"

#include "Test.h"
#include "VTKExtra.h"
#include "ReadFile.h"
#include "WriteFile.h"
#include "DataParser.h"
#include "DataFilter.h"
#include "ActionParser.h"
#include "ActionPrediction.h"

#include "OpenGLViewer.h"
#include "util.h"

#include "TestCase.h"
#include "festival.h"

// for viewer
vec3 gPosition1;
quat myQuat;

// thread locks
sem_t mutex_contactdetector, mutex_tracker, mutex_viewer, mutex_actionrecognition, mutex_tts;
sem_t lock_contactdetector, lock_tracker, lock_viewer, lock_actionrecognition, lock_facedetector;

std::string obj_model, obj_name;
std::string phrase;

cv::Mat img_rgb_global   = cv::Mat::zeros(480,640,CV_8UC3);
cv::Mat img_rgb_global_o = cv::Mat::zeros(480,640,CV_8UC3);
cv::Mat img_rgb_global_h = cv::Mat::zeros(480,640,CV_8UC3);
cv::Mat img_rgb_global_f = cv::Mat::zeros(480,640,CV_8UC3);
cv::Mat img_depth_global = cv::Mat::zeros(480,640,CV_16UC1);
cv::Mat img_cloud_global = cv::Mat::zeros(480,640,CV_32FC3);
cv::Mat img_cloud_global_c = cv::Mat::zeros(480,640,CV_32FC3);

cv::Mat mask_hand_global = cv::Mat::zeros(480,640,CV_8UC1);
cv::Mat mask_obj_global  = cv::Mat::zeros(480,640,CV_8UC1);

cv::Rect box_obj_global, box_hand_global;

bool contact_obj { false };
bool object_only { false };
int THRESHOLD = 200;

std::string PARENT		= "Data2";
std::string KB_DIR		= "kb";
std::string DATA_DIR	= "recording";
std::string EVAL		= "Scene_Moveface";
std::string RESULT		= "Result_ObjectState";

float frame_number_global { 0.0 };
cv::Vec3f single_point_face_global;

// =============================================================================
// Tracker
// =============================================================================
void* dbotthread(void* arg)
{ 

	std::string obj_name_ = obj_name;

	// Tracker initialization
	auto PFT = std::make_shared<PFTracker>();
	PFT->Build("..");

    Eigen::Vector3d p(0.0, 0.0, 0.0);
    Eigen::Quaternion<double> q(1.0, 0.0, 0.0, 0.0);

    dbot::PoseVelocityVector pose;
    pose.position() = p;
    pose.orientation().quaternion(q);

	// image variables
	cv::Mat img_rgb 		= cv::Mat::zeros(480,640,CV_8UC3);
	cv::Mat img_depth_rgb	= cv::Mat::zeros(480,640,CV_8UC3);
	cv::Mat img_depth		= cv::Mat::zeros(480,640,CV_16UC1);
	cv::Mat img_cloud		= cv::Mat::zeros(480,640,CV_32FC3);

	cv::Vec3f single_point_obj;

	int counter = 30;
	int x, y, z;

	std::string OUTPUT_RES1;
	std::string OUTPUT_RES2;
	std::string RESULT_DIR	=  PARENT + "/" + RESULT + "/" + obj_name_ + "/";
	directoryCheck(RESULT_DIR);

    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%y%m%d%H%M%S", &tstruct);
	OUTPUT_RES1 = "example_";
	OUTPUT_RES1 += buf;
	OUTPUT_RES1 += ".txt";
	OUTPUT_RES2 = "recording_";
	OUTPUT_RES2 += buf;
	OUTPUT_RES2 += ".txt";
	OUTPUT_RES1 = RESULT_DIR + OUTPUT_RES1;
	OUTPUT_RES2 = RESULT_DIR + OUTPUT_RES2;

	cv::VideoCapture kinect(CV_CAP_OPENNI2); 
	printf("Starting Kinect ...\n");

	std::ofstream myfile, write_file;
	myfile.open     (OUTPUT_RES1.c_str(), std::ios::out | std::ios::app);
	write_file.open (OUTPUT_RES2.c_str(), std::ios::out | std::ios::app);

	while(1)
	{
		sem_wait(&lock_tracker);
		sem_wait(&lock_tracker);
		sem_wait(&lock_tracker);
		sem_wait(&mutex_tracker);
		
		kinect.grab();
		kinect.retrieve(img_rgb,CV_CAP_OPENNI_BGR_IMAGE);
		kinect.retrieve(img_depth,CV_CAP_OPENNI_DEPTH_MAP);
		kinect.retrieve(img_cloud,CV_CAP_OPENNI_POINT_CLOUD_MAP);
		frame_number_global = kinect.get(CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_POS_FRAMES);
		img_rgb_global		= img_rgb.clone();
		img_rgb_global_o	= img_rgb.clone();
		img_rgb_global_h	= img_rgb.clone();
		img_rgb_global_f	= img_rgb.clone();
		img_depth_global	= img_depth.clone();
		img_cloud_global	= img_cloud.clone();
		img_cloud_global_c	= img_cloud.clone();

		sem_post(&lock_facedetector);

		if(0)
		{
			cv::Mat depth_image = cv::Mat::zeros(480,640,CV_8UC3);
			depthImaging(img_depth_rgb, img_depth);
			imshow("depth",img_depth_rgb);
			imshow("rgb",img_rgb);
			cvWaitKey(1);
		}

		if(counter > 0)
		{
			counter--;

			if(counter == 0)
			{

				#ifdef NEVER
				p[0] = single_point_obj[0];
				p[1] = single_point_obj[1];
				p[2] = single_point_obj[2];
				#endif

				std::vector<std::vector<std::string> > data_full;
				readFile(
						std::string(
								"../config/pose_cache_" +
								obj_model + ".txt").c_str(),
								data_full, ' ');

				p[0]  = std::stof(data_full[0][0]);
				p[1]  = std::stof(data_full[0][1]);
				p[2]  = std::stof(data_full[0][2]);
				q.w() = std::stof(data_full[0][3]);
				q.x() = std::stof(data_full[0][4]);
				q.y() = std::stof(data_full[0][5]);
				q.z() = std::stof(data_full[0][6]);

				pose.position() = p;
   				pose.orientation().quaternion(q);

				auto initial_poses = PFT->InitialPoses();
				initial_poses[0].component(0) = pose;
				PFT->InitialPoses(initial_poses);

				PFT->Initialize();
				printf("===================================================\n");
			}

			sem_post(&lock_tracker);
			sem_post(&lock_tracker);
		}
		else
		{
			auto poses = PFT->Track(img_depth);
			auto row =
					(PFT->ObjectIndex() / (640/PFT->DownsamplingFactor())) *
							PFT->DownsamplingFactor();
			auto col =
					(PFT->ObjectIndex() % (640/PFT->DownsamplingFactor())) *
							PFT->DownsamplingFactor();

			auto pp = poses.component(0).position();
			auto rm = poses.component(0).orientation().rotation_matrix();

			auto qu = poses.component(0).orientation().quaternion();

			auto ea = PFT->EulerAngle(rm);

			myQuat.w = qu.w();
			myQuat.x = qu.x();
			myQuat.y = qu.y();
			myQuat.z = qu.z();
			gPosition1.x = -(float)pp[0]*1.f;
			gPosition1.y = -(float)pp[1]*1.f;
			gPosition1.z =  (float)pp[2]*1.f;

			if (obj_name_ == "CUP")
			{
				z = 80;
				if(row>360) 	 y = 20;
				else 			 y = 15;
				if(col>480) 	 x = 18;
				else if(col<160) x = 8;
				else 			 x = 13;
			}
			else if (obj_name_ == "SPG")
			{
				z = 80;
				if(row>360) 	 y = 20;
				else 			 y = 15;
				if(col>480) 	 x = 15;
				else if(col<160) x = 5;
				else 			 x = 10;

			/*	z = 60;
				if(row>360) 	 y = 18;
				else 			 y = 13;
				if(col>480) 	 x = 13;
				else if(col<160) x = 3;
				else 			 x = 8;*/
			}
			else if (obj_name_ == "APP")
			{
				z = 80;
				if(row>360) 	 y = 20;
				else 			 y = 15;
				if(col>480) 	 x = 18;
				else if(col<160) x = 8;
				else 			 x = 13;
			}

			box_obj_global.x =
					col-(PFT->DownsamplingFactor()*x) < 0 ?
							0: col-(PFT->DownsamplingFactor()*x);
			box_obj_global.y =
					row-(PFT->DownsamplingFactor()*y) < 0 ?
							0: row-(PFT->DownsamplingFactor()*y);
			box_obj_global.width = box_obj_global.height = z;

			// [WRITE] *********************************************************************
			if(frame_number_global>50.0)
			{
				myfile	<< pp[0]	<< " " << pp[1]	  << " " << pp[2]	<< " "
					 	<< rm(0,0)	<< " " << rm(0,1) << " " << rm(0,2) << " "
					 	<< rm(1,0)	<< " " << rm(1,1) << " " << rm(1,2) << " "
					 	<< rm(2,0)	<< " " << rm(2,1) << " " << rm(2,2) << " "
					 	<< ea[0]	<< " " << ea[1]	  << " " << ea[2] 	<< " "
					 	<< qu.w()	<< " " << qu.x()  << " " << qu.y() 	<< " " << qu.z() << " "
					 	<< contact_obj	<< "\n";

				write_file 	<< frame_number_global << ","
							<< contact_obj << ","
							<< pp[0]   << ","
							<< pp[1]   << ","
							<< pp[2]   << ","
							<< single_point_face_global[0] << ","
							<< single_point_face_global[1] << ","
							<< single_point_face_global[2] << ","
							<< rm(0,0) << "," << rm(0,1) << "," << rm(0,2) << ","
							<< rm(1,0) << "," << rm(1,1) << "," << rm(1,2) << ","
							<< rm(2,0) << "," << rm(2,1) << "," << rm(2,2) << ","
							<< ea[0]   << "," << ea[1]   << "," << ea[2]   << ","
							<< qu.w()  << "," << qu.x()  << "," << qu.y()  << "," << qu.z()
							<< "\n";
			}
			// ********************************************************************* [WRITE]

			sem_post(&lock_contactdetector);
			sem_post(&lock_actionrecognition);
		}

		sem_post(&mutex_tracker);
		sem_post(&lock_viewer);
	}

	return 0;
}

// =============================================================================
// VIEWER
// =============================================================================
void* openglthread(void* arg)
{ 
	OpenGLViewer viewer;
 	GLFWwindow* window;
 	GLuint VertexArrayID, vertexbuffer, colorbuffer;
	GLuint vertexbufferAxes, colorbufferAxes;
	GLuint programID, MatrixID, ViewMatrixID, ModelMatrixID;
	glm::mat4 ViewMatrix, ProjectionMatrix, ModelMatrix, MVP;
	std::vector<glm::vec3> vertices;

	// Create Window
	if (1)
	{	
		// Initialise GLFW
		if( !glfwInit() )
		{
			fprintf( stderr, "Failed to initialize GLFW\n" );
			getchar();
			return 0;
		}

		glfwWindowHint(GLFW_SAMPLES, 4);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
		glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); 
		glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

		// Open a window and create its OpenGL context
		window = glfwCreateWindow(640, 480, "viewer", NULL, NULL);
		if( window == NULL ){
			fprintf(
					stderr,
					"Failed to open GLFW window. ");
			fprintf(
					stderr,
					"If you have an Intel GPU, they are not 3.3 compatible. ");
			fprintf(
					stderr,
					"Try the 2.1 version of the tutorials.\n" );
			getchar();
			glfwTerminate();
			return 0;
		}

		glfwSetWindowPos(window,700,0);

		glfwMakeContextCurrent(window);

		// Ensure we can capture the escape key being pressed below
		glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);
	
		// Enable depth test
		glEnable(GL_DEPTH_TEST);

		// Accept fragment if it closer to the camera than the former one
		glDepthFunc(GL_LESS); 

		// Dark blue background
		glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
	}

	viewer.Initialize(VertexArrayID, vertexbuffer, colorbuffer,
		vertexbufferAxes, colorbufferAxes, programID,
		MatrixID, ViewMatrixID, ModelMatrixID, ViewMatrix, ProjectionMatrix,
		vertices, obj_model.c_str());

	GLuint vertexbufferscene, colorbufferscene;
	glGenBuffers(1, &vertexbufferscene);
	glGenBuffers(1, &colorbufferscene);

	 // Check if the ESC key was pressed or the window was closed
	//while( glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS &&
	//	   glfwWindowShouldClose(window) == 0 )
	while(1)	
	{
		sem_wait(&lock_viewer);
		sem_wait(&mutex_viewer);

		// Clear the screen. 
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Use our shader
		glUseProgram(programID);
		glEnable(GL_PROGRAM_POINT_SIZE);
		
if (!object_only)
{
		// [SCENE] *********************************************************
		std::vector<glm::vec3> vertices2, verticesC;
		for(int i=0;i<img_cloud_global.size().height;i++)
		{
			for(int ii=0;ii<img_cloud_global.size().width;ii++)
			{
				glm::vec3 vertice(
						-img_cloud_global.at<cv::Vec3f>(i,ii)[0],
						 img_cloud_global.at<cv::Vec3f>(i,ii)[1],
						 img_cloud_global.at<cv::Vec3f>(i,ii)[2]);
				vertices2.push_back(vertice);
			}
		}
		for(int i=0;i<img_rgb_global.size().height;i++)
		{
			for(int ii=0;ii<img_rgb_global.size().width;ii++)
			{
				glm::vec3 vertice(
						(float)img_rgb_global.at<cv::Vec3b>(i,ii)[2]/255.0,
						(float)img_rgb_global.at<cv::Vec3b>(i,ii)[1]/255.0,
						(float)img_rgb_global.at<cv::Vec3b>(i,ii)[0]/255.0);
				verticesC.push_back(vertice);
			}
		}

		//FOR THE object
		glBindBuffer(GL_ARRAY_BUFFER, vertexbufferscene);
		glBufferData(GL_ARRAY_BUFFER, vertices2.size() * sizeof(glm::vec3), &vertices2[0], GL_STATIC_DRAW);

		//For the Color
		glBindBuffer(GL_ARRAY_BUFFER, colorbufferscene);
		glBufferData(GL_ARRAY_BUFFER, verticesC.size() * sizeof(glm::vec3), &verticesC[0], GL_STATIC_DRAW);

		// 1rst attribute buffer : vertices
		glEnableVertexAttribArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, vertexbufferscene);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

		// 2nd attribute buffer : colors
		glEnableVertexAttribArray(1);
		glBindBuffer(GL_ARRAY_BUFFER, colorbufferscene);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

		ModelMatrix = 
				scale(mat4(), vec3(1.f, 1.f, 1.f));
		MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;

		glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);
		glUniformMatrix4fv(ModelMatrixID, 1, GL_FALSE, &ModelMatrix[0][0]);
		glUniformMatrix4fv(ViewMatrixID, 1, GL_FALSE, &ViewMatrix[0][0]);

		// Draw the triangle !
		glDrawArrays(GL_POINTS, 0, vertices2.size() );
		// ********************************************************* [SCENE]
}

		// [OBJ] ***********************************************************
		// 1rst attribute buffer : vertices
		glEnableVertexAttribArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

		// 2nd attribute buffer : colors
		glEnableVertexAttribArray(1);
		glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

		ModelMatrix =
				glm::translate(mat4(), gPosition1) * 
				glm::toMat4(myQuat) *
				scale(mat4(), vec3(1.f, 1.f, 1.f));
		MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;

		glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);
		glUniformMatrix4fv(ModelMatrixID, 1, GL_FALSE, &ModelMatrix[0][0]);
		glUniformMatrix4fv(ViewMatrixID, 1, GL_FALSE, &ViewMatrix[0][0]);

		// Draw the triangle !
		glDrawArrays(GL_TRIANGLES, 0, vertices.size() );
		// *********************************************************** [OBJ]

		// [LINE] **********************************************************
		// 1rst attribute buffer : vertices
		glEnableVertexAttribArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, vertexbufferAxes);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

		// 2nd attribute buffer : colors
		glEnableVertexAttribArray(1);
		glBindBuffer(GL_ARRAY_BUFFER, colorbufferAxes);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

		glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);
		glDrawArrays(GL_LINES, 0, 6*3);
		// ********************************************************** [LINE]

		glDisableVertexAttribArray(0);
		glDisableVertexAttribArray(1);
	
		// Swap buffers
		glfwSwapBuffers(window);
		glfwPollEvents();

		sem_post(&mutex_viewer);
		sem_post(&lock_tracker);
	}
	return 0;
}

// ============================================================================
// OBJECT DETECTOR
// ============================================================================
void* objectDetector(void* arg)
{
	int hs[4]; // hue max/min, sat max/min
	hs[0] = 98; hs[1] = 90; hs[2] = 230; hs[3] = 140; // yellow sponge
	hs[0] = 27; hs[1] =  0; hs[2] = 186; hs[3] = 105; // blue cup
	hs[0] = 18; hs[1] =  0; hs[2] = 122; hs[3] =  59; // blue cup
	hs[0] = 90; hs[1] = 72; hs[2] = 179; hs[3] = 84; // green cup

	while(true)
	{
		sem_wait(&lock3);
		sem_wait(&mutex3);

		segmentHSV(img_rgb_global_o, hs, mask_obj_global, box_obj_global);

		if(0)
		{
			cv::Mat rgb_tmp = cv::Mat::zeros(480,640, CV_8UC3);
			img_rgb_global_o.copyTo(rgb_tmp, mask_obj_global);
			cv::imshow("rgb_o",rgb_tmp);
			cvWaitKey(1);
		}

		sem_post(&mutex3);
		sem_post(&lock1);
		sem_post(&lock5);
	}

	return 0;
}

// ============================================================================
// HAND DETECTOR
// ============================================================================
void* handDetector(void* arg)
{ 
	int hs[4]; // hue max/min, sat max/min
	hs[0] = 107; hs[1] = 90; hs[2] = 107; hs[3] = 66;
	hs[0] = 125; hs[1] = 98; hs[2] = 140; hs[3] = 77;
	hs[0] = 107; hs[1] = 98; hs[2] = 140; hs[3] = 77;

	cv::Mat img_no_head = cv::Mat::zeros(480,640,CV_8UC3);

	while(true)
	{
		sem_wait(&lock4);
		sem_wait(&mutex4);

      	//rgb_global3.clone().rowRange(200,480).copyTo(img_no_head.rowRange(200,480));
		segmentHSV(img_rgb_global_h, hs, mask_hand_global, box_hand_global);

		if(0)
		{
			cv::Mat rgb_tmp = cv::Mat::zeros(480,640, CV_8UC3);
			img_rgb_global_h.copyTo(rgb_tmp, mask_hand_global);
			cv::imshow("rgb_h",rgb_tmp);
			cvWaitKey(1);
		}

		sem_post(&mutex4);
		sem_post(&lock1);
		sem_post(&lock5);
	}

	return 0;
}

// ============================================================================
// CONTACT DETECTOR
// ============================================================================
void* contactDetector(void* arg)
{
	while(1)
	{
		sem_wait(&lock_contactdetector);
		sem_wait(&mutex_contactdetector);
		cv::Mat img_rgb_tmp = cv::Mat::zeros(480,640,CV_8UC3);
		img_rgb_global_o(box_obj_global).copyTo(img_rgb_tmp); // reducing the search area
		contact_obj = contactCheckBox(img_rgb_tmp, "rgb_c", THRESHOLD, false);
		sem_post(&mutex_contactdetector);
		sem_post(&lock_tracker);
	}


	cv::Mat img_depth_def, mask_obj_def, img_sub, cloud_mask,cloud_mask2;
	double contact_val;
	bool flag_contact_init	= true;
	bool flag_contact_obj	= false;
	int c = 0;
	while(true)
	{
		sem_wait(&lock5);
		sem_wait(&lock5);
		sem_wait(&mutex5);

		// [DEFAULT SCENE]*****************************************************
		if(flag_contact_init)
		{
			mask_obj_def = mask_obj_global.clone();
			img_depth_global.copyTo(img_depth_def,mask_obj_def);
			if(box_obj_global.x > 0 && box_obj_global.y > 0) 
				flag_contact_init = false;
		}
		// *****************************************************[DEFAULT SCENE]

		// [OBJECT CONTACT]****************************************************
		if(contactCheck(mask_hand_global, mask_obj_global,
						box_hand_global, box_obj_global))
		{
			if(!flag_contact_obj)
			{
				img_depth_global.copyTo(img_sub,mask_obj_def);			
				absdiff(img_depth_def, img_sub, img_sub);
				contact_val = sum(img_sub)[0] / sum(mask_obj_def)[0];

				if(contact_val > 0 && contact_val < 500)
				{
					contact_obj 		= true;
					flag_contact_obj 	= true;
				}
				else contact_obj = false;
			}
			else contact_obj = true;     
		}
		else
		{
			flag_contact_init 	= true;
			flag_contact_obj 	= false;
			contact_obj 		= false;
			contact_val			= 0.0;
		}
		// face prevention
		if(box_obj_global.tl().x > 320) 
		{
			flag_contact_init	= false;
			flag_contact_obj	= true;
			contact_obj 		= true;
			contact_val			= -1.0;
		} 
		// ****************************************************[OBJECT CONTACT]

		sem_post(&mutex5);
		sem_post(&lock1);
	}

	return 0;
}

// ============================================================================
// FACE DETECTOR
// ============================================================================
void* faceDetector(void* arg)
{
	//Load the cascade for face detector
	std::string face_cascade_name = "../cascade/lbpcascade_frontalface.xml";
	cv::CascadeClassifier face_cascade;
	if(!face_cascade.load(face_cascade_name))
	printf("--(!)Error loading face cascade\n");

	cv::Mat cloud_mask, img_tmp;
	cv::Rect face;

	bool NANflag = true;

	while(true)
	{
		sem_wait(&lock_facedetector);
		sem_wait(&mutex6);

		// just to flush out some frames
		if (frame_number_global < 50.0 || NANflag)
		{
			if (!(countNonZero(img_rgb_global_f!=img_tmp) == 0))
			{	
				img_tmp.release();
				img_tmp = img_rgb_global_f.clone();
				face = detectFaceAndEyes(img_rgb_global_f, face_cascade);
				//[OBJECT POINT]***************************************************
				img_cloud_global_c(face).copyTo(cloud_mask); // reducing the search area
				pointCloudTrajectory(cloud_mask, single_point_face_global);
				NANflag = 
					(isnan(single_point_face_global[0]) ||
					 isnan(single_point_face_global[1]) ||
					 isnan(single_point_face_global[2]));
				cloud_mask.release(); 
				// **************************************************[OBJECT POINT]
			}
		}
		else continue;

		sem_post(&mutex6);
	}
	return 0;
}

// ============================================================================
// ACTION RECOGNITION
// ============================================================================
void* actionRecognition(void* arg)
{
	// [VARIABLE] **************************************************************
	std::shared_ptr<CKB> KB = std::make_shared<CKB>();
	std::shared_ptr<COS> OS = std::make_shared<COS>();
	std::shared_ptr<std::vector<std::string> > message = std::make_shared<std::vector<std::string> >();

	std::string phrase_now	= "";
	std::string path;
	std::string PARSED_RES;
	std::string OUTPUT_RES;
	std::string object		= obj_name;
	std::string RESULT_DIR	=  PARENT + "/" + RESULT + "/" + object + "/";
	directoryCheck(RESULT_DIR);

    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%y%m%d%H%M%S", &tstruct);
	PARSED_RES = "parsed_output_";
	PARSED_RES += buf;
	PARSED_RES += ".txt";
	OUTPUT_RES = "output_";
	OUTPUT_RES += buf;
	OUTPUT_RES += ".txt";

	int loc_int = 100;
	int sec_int = 36;
	int filter_w = 5;

	bool nolabel = false;
	bool gauss = true;

	ReadFile RF;
	// ************************************************************** [VARIABLE] 

	/* Reading surface
	 * Reading action labels
	 * - reads the labels and initializes a zero list prediction/filter with the same length as the label
	 * Reading object specific labels
	 * - reads the object specific labels and saves them */
	path = PARENT + "/" + KB_DIR + "/";
	if (RF.ReadFileKB(path, KB)==EXIT_FAILURE)
	{return 0;}

	/* read object state if needed */
	path = PARENT + "/" + KB_DIR + "/";
	if (RF.ReadFileOS(path, OS)==EXIT_FAILURE)
	{return 0;}

	/* read parse message */
	path = PARENT + "/" + KB_DIR + "/";
	if (RF.ReadMsg(path, message)==EXIT_FAILURE)
	{return 0;}

	// directory to parsed message
	path = PARENT + "/" + RESULT + "/" + object + "/";
	directoryCheck(path);

	auto T =
			std::make_shared<Test>(
					object, loc_int, sec_int, filter_w, KB, OS, message,
					path, true);

	// directory of learned data of a subject
	std::string dir_s = PARENT + "/" + EVAL + "/" + object + "/0";

	// read available location areas
	path  = dir_s + "/location_area.txt";
	if (T->ReadLA(path)==EXIT_FAILURE) {return 0;}

	// read available sector map
	path = 	dir_s + "/graph.txt";
	if (T->ReadGraph(path)==EXIT_FAILURE) {return 0;}

	// apply gauss filter
	if (gauss) { T->ApplyGauss(5,5); }

	// write window constraint
	path = 	dir_s + "/window.txt";
	if (T->WriteWindow(path)==EXIT_FAILURE) {return 0;}

	// Initialize
	T->TestInit();

	int c = 0;
	bool flag = true;
	while(true)
	{
		sem_wait(&lock_actionrecognition);
		sem_wait(&mutex_actionrecognition);

		if ( isnan(single_point_face_global[0])==false &&
			 isnan(single_point_face_global[1])==false &&
			 isnan(single_point_face_global[2])==false && flag)
		{
			Eigen::Vector4d face_parser(
					 single_point_face_global[0],
					-single_point_face_global[1],
					 single_point_face_global[2],
					 0.0);
			//if (object=="CUP")
			face_parser += Eigen::Vector4d(-0.1, 0, 0.1, -1);
			T->TestFaceAdjust(face_parser);
			flag = false;
		}

		// 1. Filter
		T->FilterData(
				Eigen::Vector4d(-gPosition1.x, -gPosition1.y, gPosition1.z, -2.0),
				(int)contact_obj);

		// 2. Prediction
		T->Predict();

		// 3. Get Data
		T->GetData((int)frame_number_global);

		// 4. Parse Data
		T->Parser(PARSED_RES, (int)frame_number_global, phrase_now);		
		phrase = phrase_now;

		c++;
		if (c==30)
		{
			c = 0;
			// 5. Write Results
			T->WriteResult(OUTPUT_RES, RESULT_DIR, true);
		}

		sem_post(&mutex_actionrecognition);
		sem_post(&lock_tracker);
	}
	return 0;
}

// ============================================================================
// TTS
// ============================================================================
void* tts(void* arg)
{
	std::string phrase_last	= "";
	int heap_size = 210000;  // default scheme heap size
	int load_init_files = 1; // we want the festival init files loaded
	festival_initialize(load_init_files,heap_size);
	festival_eval_command("(voice_ked_diphone)");

	while(1)
	{
		sem_wait(&mutex_tts);
		if (phrase != phrase_last)
		{
			phrase_last = phrase;
			std::cout << phrase_last << std::endl;
			festival_say_text(phrase_last.c_str());
		}
		sem_post(&mutex_tts);
	}
	return 0;
}

// =============================================================================
// >>>>> MAIN <<<<<
// =============================================================================

int main(int argc, char *argv[])
{

	if (argc==1)
	{
		std::cerr << "Object file not given." << std::endl;
		std::cerr << "Exiting..............." << std::endl;
		return 0;
	}
	else if (argc==2)
	{
		std::cerr << "Object model file not given.                  " << std::endl;
		std::cerr << "File should be in ../config/pose_cache_xxx.txt" << std::endl;
		std::cerr << "Exiting......................................." << std::endl;
		return 0;
	}
	else if (argc==3)
	{
		std::cerr << "Threshold for hand contact not given. (200-500)" << std::endl;
		std::cerr << "Exiting........................................" << std::endl;
		return 0;
	}
	else if (argc==4)
	{
		obj_model = std::string(argv[1]);
		obj_name  = std::string(argv[2]);
		std::cout << "Using object model file : " << obj_model << std::endl;
		THRESHOLD = std::stoi(argv[3]);
	}
	else
	{
		std::cerr << "Too much arguments given." << std::endl;
		std::cerr << "Exiting.................." << std::endl;
		return 0;
	}

//	if (obj_name == "CUP") 	 	THRESHOLD = 250;
//	else if (obj_name == "SPG")	THRESHOLD = 200;
//	else if (obj_name == "APP")	THRESHOLD = 300;

//	cv::namedWindow("rgb");		cv::moveWindow("rgb",0,550);  
//	cv::namedWindow("depth");	cv::moveWindow("depth",0,0);
//	cv::namedWindow("rgb_o");	cv::moveWindow("rgb_o",0,550);  
//	cv::namedWindow("rgb_h");	cv::moveWindow("rgb_h",0,0);
//	cv::namedWindow("rgb_c");	cv::moveWindow("rgb_c",0,0);

	sem_init(&lock_tracker, 0, 3);
	sem_init(&lock_viewer, 0, 0);
	sem_init(&lock_contactdetector, 0, 0);
	sem_init(&lock_actionrecognition, 0, 0);
	sem_init(&lock_facedetector, 0, 0);

	sem_init(&mutex_tracker, 0, 1);
	sem_init(&mutex_viewer, 0, 1);
	sem_init(&mutex_contactdetector, 0, 1);
	sem_init(&mutex_actionrecognition, 0, 1);
	sem_init(&mutex_tts, 0, 1);

	pthread_t 	thread_dbot,
				thread_opengl,
				thread_faceDetector,
				thread_contactDetector,
				thread_actionrecognition,
				thread_tts;

	pthread_attr_t attr;
	cpu_set_t cpus;
	pthread_attr_init(&attr);

	CPU_ZERO(&cpus);
	CPU_SET(1, &cpus);
	pthread_attr_setaffinity_np(&attr, sizeof(cpu_set_t), &cpus);
	pthread_create(&thread_dbot, &attr, dbotthread, NULL);

	CPU_ZERO(&cpus);
	CPU_SET(2, &cpus);
	pthread_attr_setaffinity_np(&attr, sizeof(cpu_set_t), &cpus);
	pthread_create(&thread_opengl, &attr, openglthread, NULL);

//	CPU_ZERO(&cpus);
//	CPU_SET(3, &cpus);
//	pthread_attr_setaffinity_np(&attr, sizeof(cpu_set_t), &cpus);
//	pthread_create(&thread_obj, &attr, objectDetector, NULL);

//	CPU_ZERO(&cpus);
//	CPU_SET(4, &cpus);
//	pthread_attr_setaffinity_np(&attr, sizeof(cpu_set_t), &cpus);
//	pthread_create(&thread_hand, &attr, handDetector, NULL);

	CPU_ZERO(&cpus);
	CPU_SET(3, &cpus);
	pthread_attr_setaffinity_np(&attr, sizeof(cpu_set_t), &cpus);
	pthread_create(&thread_contactDetector, &attr, contactDetector, NULL);

	CPU_ZERO(&cpus);
	CPU_SET(4, &cpus);
	pthread_attr_setaffinity_np(&attr, sizeof(cpu_set_t), &cpus);
	pthread_create(&thread_faceDetector, &attr, faceDetector, NULL);

	CPU_ZERO(&cpus);
	CPU_SET(5, &cpus);
	pthread_attr_setaffinity_np(&attr, sizeof(cpu_set_t), &cpus);
	pthread_create(&thread_actionrecognition, &attr, actionRecognition, NULL);

	CPU_ZERO(&cpus);
	CPU_SET(6, &cpus);
	pthread_attr_setaffinity_np(&attr, sizeof(cpu_set_t), &cpus);
	pthread_create(&thread_tts, &attr, tts, NULL);

	pthread_join(thread_dbot, NULL);
	pthread_join(thread_opengl, NULL);
//	pthread_join(thread_obj, NULL);
//	pthread_join(thread_hand, NULL);
	pthread_join(thread_contactDetector, NULL);
	pthread_join(thread_faceDetector, NULL);
	pthread_join(thread_actionrecognition, NULL);
	pthread_join(thread_tts, NULL);


	return 0;
}





















