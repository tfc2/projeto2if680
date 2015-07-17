#include "Main.h"
#include "util.h"
#include <math.h>
#include <Windows.h>
#include <math.h>

// OpenCV includes
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"

#include <iostream>
#include <ctype.h>

/************************************************************************
Window
************************************************************************/

typedef struct {
	int width;
	int height;
	char* title;

	float field_of_view_angle;
	float z_near;
	float z_far;
} glutWindow;

/***************************************************************************
OBJ Loading
***************************************************************************/

class Model_OBJ
{
public:
	Model_OBJ();
	float* Model_OBJ::calculateNormal(float* coord1, float* coord2, float* coord3);
	int Model_OBJ::Load(char *filename);	// Loads the model
	void Model_OBJ::Draw();					// Draws the model on the screen
	void Model_OBJ::Release();				// Release the model

	float* normals;							// Stores the normals
	float* Faces_Triangles;					// Stores the triangles
	float* vertexBuffer;					// Stores the points which make the object
	long TotalConnectedPoints;				// Stores the total number of connected verteces
	long TotalConnectedTriangles;			// Stores the total number of connected triangles

};


#define POINTS_PER_VERTEX 3
#define TOTAL_FLOATS_IN_TRIANGLE 9
using namespace std;



//////////////////////////////////////////////////////////////////////////////////////////////////////////////
const double plane_size = 30.0;                 // Extent of plane
const double plane_level = -10;               // Level (y-coord) of plane
GLdouble planepts[4][3] = {                    // Corners of "plane"
	{ -plane_size, plane_level, -plane_size },
	{ -plane_size, plane_level, plane_size },
	{ plane_size, plane_level, plane_size },
	{ plane_size, plane_level, -plane_size } };

GLdouble plane_eq[4];                          // Holds plane "equation"
//  in the form Ax+By+Cz=D
GLdouble shadowmat[16];                        // Shadow projection matrix

// makeshadowmatrix
// Create a matrix that will project the desired shadow.
// Is given the plane equation, in the form Ax+By+Cz=D.
// Is given light postion, in homogeneous form.
// Creates matrix in shadowmat.
void makeshadowmatrix(GLdouble shadowmat[16],
	const GLdouble plane_eq[4],
	const GLdouble lightpos[4])
{
	// Find dot product between light position vector and ground plane normal.
	GLdouble dot = plane_eq[0] * lightpos[0] +
		plane_eq[1] * lightpos[1] +
		plane_eq[2] * lightpos[2] +
		plane_eq[3] * lightpos[3];

	typedef GLdouble(*mat44)[4];
	mat44 sm = reinterpret_cast<mat44>(shadowmat);

	sm[0][0] = dot - lightpos[0] * plane_eq[0];
	sm[1][0] = -lightpos[0] * plane_eq[1];
	sm[2][0] = -lightpos[0] * plane_eq[2];
	sm[3][0] = -lightpos[0] * plane_eq[3];

	sm[0][1] = -lightpos[1] * plane_eq[0];
	sm[1][1] = dot - lightpos[1] * plane_eq[1];
	sm[2][1] = -lightpos[1] * plane_eq[2];
	sm[3][1] = -lightpos[1] * plane_eq[3];

	sm[0][2] = -lightpos[2] * plane_eq[0];
	sm[1][2] = -lightpos[2] * plane_eq[1];
	sm[2][2] = dot - lightpos[2] * plane_eq[2];
	sm[3][2] = -lightpos[2] * plane_eq[3];

	sm[0][3] = -lightpos[3] * plane_eq[0];
	sm[1][3] = -lightpos[3] * plane_eq[1];
	sm[2][3] = -lightpos[3] * plane_eq[2];
	sm[3][3] = dot - lightpos[3] * plane_eq[3];
}


// findplane
// Calculate A,B,C,D version of plane equation,
// given 3 non-colinear points in plane
void findplane(GLdouble plane_eq[4],
	const GLdouble p0[3],
	const GLdouble p1[3],
	const GLdouble p2[3])
{
	GLdouble vec0[3], vec1[3];

	// Need 2 vectors to find cross product.
	vec0[0] = p1[0] - p0[0];
	vec0[1] = p1[1] - p0[1];
	vec0[2] = p1[2] - p0[0];

	vec1[0] = p2[0] - p0[0];
	vec1[1] = p2[1] - p0[1];
	vec1[2] = p2[2] - p0[2];

	// find cross product to get A, B, and C of plane equation
	plane_eq[0] = vec0[1] * vec1[2] - vec0[2] * vec1[1];
	plane_eq[1] = vec0[2] * vec1[0] - vec0[0] * vec1[2];
	plane_eq[2] = vec0[0] * vec1[1] - vec0[1] * vec1[0];

	GLdouble normlen = sqrt(plane_eq[0] * plane_eq[0]
		+ plane_eq[1] * plane_eq[1]
		+ plane_eq[2] * plane_eq[2]);
	if (normlen != 0)
	{
		plane_eq[0] /= normlen;
		plane_eq[1] /= normlen;
		plane_eq[2] /= normlen;
	}
	else
	{
		plane_eq[0] = 1.;
		plane_eq[1] = 0.;
		plane_eq[2] = 0.;
	}

	plane_eq[3] = -(plane_eq[0] * p0[0]
		+ plane_eq[1] * p0[1]
		+ plane_eq[2] * p0[2]);
}

Model_OBJ::Model_OBJ()
{
	this->TotalConnectedTriangles = 0;
	this->TotalConnectedPoints = 0;
}

float* Model_OBJ::calculateNormal(float *coord1, float *coord2, float *coord3)
{
	/* calculate Vector1 and Vector2 */
	float va[3], vb[3], vr[3], val;
	va[0] = coord1[0] - coord2[0];
	va[1] = coord1[1] - coord2[1];
	va[2] = coord1[2] - coord2[2];

	vb[0] = coord1[0] - coord3[0];
	vb[1] = coord1[1] - coord3[1];
	vb[2] = coord1[2] - coord3[2];

	/* cross product */
	vr[0] = va[1] * vb[2] - vb[1] * va[2];
	vr[1] = vb[0] * va[2] - va[0] * vb[2];
	vr[2] = va[0] * vb[1] - vb[0] * va[1];

	/* normalization factor */
	val = sqrt(vr[0] * vr[0] + vr[1] * vr[1] + vr[2] * vr[2]);

	float norm[3];
	norm[0] = vr[0] / val;
	norm[1] = vr[1] / val;
	norm[2] = vr[2] / val;


	return norm;
}


int Model_OBJ::Load(char* filename)
{
	string line;
	ifstream objFile(filename);
	if (objFile.is_open())													// If obj file is open, continue
	{
		objFile.seekg(0, ios::end);										// Go to end of the file, 
		long fileSize = objFile.tellg();									// get file size
		objFile.seekg(0, ios::beg);										// we'll use this to register memory for our 3d model

		vertexBuffer = (float*)malloc(fileSize);							// Allocate memory for the verteces
		Faces_Triangles = (float*)malloc(fileSize*sizeof(float));			// Allocate memory for the triangles
		normals = (float*)malloc(fileSize*sizeof(float));					// Allocate memory for the normals

		int triangle_index = 0;												// Set triangle index to zero
		int normal_index = 0;												// Set normal index to zero

		while (!objFile.eof())											// Start reading file data
		{
			getline(objFile, line);											// Get line from file

			if (line.c_str()[0] == 'v')										// The first character is a v: on this line is a vertex stored.
			{
				line[0] = ' ';												// Set first character to 0. This will allow us to use sscanf

				sscanf(line.c_str(), "%f %f %f",							// Read floats from the line: v X Y Z
					&vertexBuffer[TotalConnectedPoints],
					&vertexBuffer[TotalConnectedPoints + 1],
					&vertexBuffer[TotalConnectedPoints + 2]);

				TotalConnectedPoints += POINTS_PER_VERTEX;					// Add 3 to the total connected points
			}


			if ((line[0] == 'f') && (line.find_first_of('/') == -1))		// The first character is an 'f': on this line is a point stored
			{
				line[0] = ' ';												// Set first character to 0. This will allow us to use sscanf

				int vertexNumber[4] = { 0, 0, 0 };

				sscanf(line.c_str(), "%d %d %d",						    // Read integers from the line:  f 1 2 3
					&vertexNumber[0],										// First point of our triangle. This is an 
					&vertexNumber[1],										// pointer to our vertexBuffer list
					&vertexNumber[2]);										// each point represents an X,Y,Z.

				vertexNumber[0] -= 1;										// OBJ file starts counting from 1
				vertexNumber[1] -= 1;										// OBJ file starts counting from 1
				vertexNumber[2] -= 1;										// OBJ file starts counting from 1


				/********************************************************************
				* Create triangles (f 1 2 3) from points: (v X Y Z) (v X Y Z) (v X Y Z).
				* The vertexBuffer contains all verteces
				* The triangles will be created using the verteces we read previously
				*/

				int tCounter = 0;
				for (int i = 0; i < POINTS_PER_VERTEX; i++)
				{
					Faces_Triangles[triangle_index + tCounter] = vertexBuffer[3 * vertexNumber[i]];
					Faces_Triangles[triangle_index + tCounter + 1] = vertexBuffer[3 * vertexNumber[i] + 1];
					Faces_Triangles[triangle_index + tCounter + 2] = vertexBuffer[3 * vertexNumber[i] + 2];
					tCounter += POINTS_PER_VERTEX;
				}

				/*********************************************************************
				* Calculate all normals, used for lighting
				*/
				float coord1[3] = { Faces_Triangles[triangle_index], Faces_Triangles[triangle_index + 1], Faces_Triangles[triangle_index + 2] };
				float coord2[3] = { Faces_Triangles[triangle_index + 3], Faces_Triangles[triangle_index + 4], Faces_Triangles[triangle_index + 5] };
				float coord3[3] = { Faces_Triangles[triangle_index + 6], Faces_Triangles[triangle_index + 7], Faces_Triangles[triangle_index + 8] };
				float *norm = this->calculateNormal(coord1, coord2, coord3);

				tCounter = 0;
				for (int i = 0; i < POINTS_PER_VERTEX; i++)
				{
					normals[normal_index + tCounter] = norm[0];
					normals[normal_index + tCounter + 1] = norm[1];
					normals[normal_index + tCounter + 2] = norm[2];
					tCounter += POINTS_PER_VERTEX;
				}

				triangle_index += TOTAL_FLOATS_IN_TRIANGLE;
				normal_index += TOTAL_FLOATS_IN_TRIANGLE;
				TotalConnectedTriangles += TOTAL_FLOATS_IN_TRIANGLE;
			}
			else if (line[0] == 'f') { // entrada do tipo n/n/n/ n/n/n/ n/n/n/

				line[0] = ' ';												// Set first character to 0. This will allow us to use sscanf

				int vertexNumber[4] = { 0, 0, 0 };

				line = line.substr((line.find_first_of(' ') + 1), line.length());
				line = line.substr((line.find_first_of(' ') + 1), line.length());

				string n1 = line.substr(0, (line.find_first_of('/')));

				vertexNumber[0] = atoi(n1.c_str());

				line = line.substr((line.find_first_of(' ') + 1), line.length());

				string n2 = line.substr(0, (line.find_first_of('/')));

				vertexNumber[1] = atoi(n2.c_str());

				line = line.substr((line.find_first_of(' ') + 1), line.length());

				string n3 = line.substr(0, (line.find_first_of('/')));

				vertexNumber[2] = atoi(n3.c_str());

				vertexNumber[0] -= 1;										// OBJ file starts counting from 1
				vertexNumber[1] -= 1;										// OBJ file starts counting from 1
				vertexNumber[2] -= 1;										// OBJ file starts counting from 1

				/********************************************************************
				* Create triangles (f 1 2 3) from points: (v X Y Z) (v X Y Z) (v X Y Z).
				* The vertexBuffer contains all verteces
				* The triangles will be created using the verteces we read previously
				*/

				int tCounter = 0;
				for (int i = 0; i < POINTS_PER_VERTEX; i++)
				{
					Faces_Triangles[triangle_index + tCounter] = vertexBuffer[3 * vertexNumber[i]];
					Faces_Triangles[triangle_index + tCounter + 1] = vertexBuffer[3 * vertexNumber[i] + 1];
					Faces_Triangles[triangle_index + tCounter + 2] = vertexBuffer[3 * vertexNumber[i] + 2];
					tCounter += POINTS_PER_VERTEX;
				}

				/*********************************************************************
				* Calculate all normals, used for lighting
				*/
				float coord1[3] = { Faces_Triangles[triangle_index], Faces_Triangles[triangle_index + 1], Faces_Triangles[triangle_index + 2] };
				float coord2[3] = { Faces_Triangles[triangle_index + 3], Faces_Triangles[triangle_index + 4], Faces_Triangles[triangle_index + 5] };
				float coord3[3] = { Faces_Triangles[triangle_index + 6], Faces_Triangles[triangle_index + 7], Faces_Triangles[triangle_index + 8] };
				float *norm = this->calculateNormal(coord1, coord2, coord3);

				tCounter = 0;
				for (int i = 0; i < POINTS_PER_VERTEX; i++)
				{
					normals[normal_index + tCounter] = norm[0];
					normals[normal_index + tCounter + 1] = norm[1];
					normals[normal_index + tCounter + 2] = norm[2];
					tCounter += POINTS_PER_VERTEX;
				}

				triangle_index += TOTAL_FLOATS_IN_TRIANGLE;
				normal_index += TOTAL_FLOATS_IN_TRIANGLE;
				TotalConnectedTriangles += TOTAL_FLOATS_IN_TRIANGLE;

				if (line.find_first_of(' ') != -1){

					line = line.substr((line.find_first_of(' ') + 1), line.length());

					string n4 = line.substr(0, (line.find_first_of('/')));

					vertexNumber[1] = atoi(n4.c_str());

					vertexNumber[1] -= 1;										// OBJ file starts counting from 1

					int tCounter = 0;

					for (int i = 0; i < POINTS_PER_VERTEX; i++)
					{
						Faces_Triangles[triangle_index + tCounter] = vertexBuffer[3 * vertexNumber[i]];
						Faces_Triangles[triangle_index + tCounter + 1] = vertexBuffer[3 * vertexNumber[i] + 1];
						Faces_Triangles[triangle_index + tCounter + 2] = vertexBuffer[3 * vertexNumber[i] + 2];
						tCounter += POINTS_PER_VERTEX;
					}

					/*********************************************************************
					* Calculate all normals, used for lighting
					*/
					float coord1[3] = { Faces_Triangles[triangle_index], Faces_Triangles[triangle_index + 1], Faces_Triangles[triangle_index + 2] };
					float coord2[3] = { Faces_Triangles[triangle_index + 3], Faces_Triangles[triangle_index + 4], Faces_Triangles[triangle_index + 5] };
					float coord3[3] = { Faces_Triangles[triangle_index + 6], Faces_Triangles[triangle_index + 7], Faces_Triangles[triangle_index + 8] };
					float *norm = this->calculateNormal(coord1, coord2, coord3);

					tCounter = 0;
					for (int i = 0; i < POINTS_PER_VERTEX; i++)
					{
						normals[normal_index + tCounter] = norm[0];
						normals[normal_index + tCounter + 1] = norm[1];
						normals[normal_index + tCounter + 2] = norm[2];
						tCounter += POINTS_PER_VERTEX;
					}

					triangle_index += TOTAL_FLOATS_IN_TRIANGLE;
					normal_index += TOTAL_FLOATS_IN_TRIANGLE;
					TotalConnectedTriangles += TOTAL_FLOATS_IN_TRIANGLE;

				}
			}
		}
		objFile.close();														// Close OBJ file
	}
	else
	{
		cout << "Unable to open file";
	}
	return 0;
}

void Model_OBJ::Release()
{
	free(this->Faces_Triangles);
	free(this->normals);
	free(this->vertexBuffer);
}

void Model_OBJ::Draw()
{
	glEnableClientState(GL_VERTEX_ARRAY);						// Enable vertex arrays
	glEnableClientState(GL_NORMAL_ARRAY);						// Enable normal arrays
	glVertexPointer(3, GL_FLOAT, 0, Faces_Triangles);			// Vertex Pointer to triangle array
	glNormalPointer(GL_FLOAT, 0, normals);						// Normal pointer to normal array
	glDrawArrays(GL_TRIANGLES, 0, TotalConnectedTriangles);		// Draw the triangles
	glDisableClientState(GL_VERTEX_ARRAY);						// Disable vertex arrays
	glDisableClientState(GL_NORMAL_ARRAY);						// Disable normal arrays
}

using namespace cv;

cv::VideoCapture cap;
Mat frame;

Model_OBJ obj;

GLfloat angle, fAspect;

Mat img_object;

glutWindow win;

// Indica os fps
int frames;
float times, timebase;

void printFPS(){
	frames++;
	times = glutGet(GLUT_ELAPSED_TIME);
	if (times - timebase > 1000) {
		printf("FPS: %4.2lf\n",
			(double)frames*1000.0 / ((double)(times - timebase)));
		timebase = times;
		frames = 0;
	}
}

// Mapeia o objeto no video
void surf()
{
	while (true) {

		Mat img_scene(100, 250, CV_8UC1, Scalar(0));

		bool bSuccess = cap.read(img_scene); // le os frames do video

		if (!bSuccess) // caso nao tenha lido, da um break
		{
			cout << "Cannot read a frame from video stream" << endl;
			break;
		}

		Size size(600, 300);
		resize(img_scene, img_scene, size);

		// Detecta pontos usando o algoritmo surf
		int minHessian = 1000;

		SurfFeatureDetector detector(minHessian);

		std::vector<KeyPoint> keypoints_object, keypoints_scene;

		detector.detect(img_object, keypoints_object);
		detector.detect(img_scene, keypoints_scene);

		// Calcula os vetores das featuras
		SurfDescriptorExtractor extractor;

		Mat descriptors_object, descriptors_scene;

		extractor.compute(img_object, keypoints_object, descriptors_object);
		extractor.compute(img_scene, keypoints_scene, descriptors_scene);

		// Faz o matching usando o algoritmo flann
		FlannBasedMatcher matcher;
		std::vector< DMatch > matches;
		matcher.match(descriptors_object, descriptors_scene, matches);

		double max_dist = 0; double min_dist = 100;

		// Calcula as distancias maximas e minimasentre os pontos
		for (int i = 0; i < descriptors_object.rows; i++)
		{
			double dist = matches[i].distance;
			if (dist < min_dist) min_dist = dist;
			if (dist > max_dist) max_dist = dist;
		}

		printFPS();

		//-- Desenha apenas bons matches (distancia menor que 2*min_dist)
		std::vector< DMatch > good_matches;

		for (int i = 0; i < descriptors_object.rows; i++)
		{
			if (matches[i].distance < 2 * min_dist)
			{
				good_matches.push_back(matches[i]);
			}
		}

		Mat img_matches;
		drawMatches(img_object, keypoints_object, img_scene, keypoints_scene,
			good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
			vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

		//-- Localiza o ojeto
		std::vector<Point2f> obj;
		std::vector<Point2f> scene;

		for (int i = 0; i < good_matches.size(); i++)
		{
			//-- Pega os pontos bons do objeto
			obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
			scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
		}

		Mat H = findHomography(obj, scene, CV_RANSAC); 

		//-- Pega as quinas do objeto a ser detectado
		std::vector<Point2f> obj_corners(4);
		obj_corners[0] = cvPoint(0, 0); obj_corners[1] = cvPoint(img_object.cols, 0);
		obj_corners[2] = cvPoint(img_object.cols, img_object.rows); obj_corners[3] = cvPoint(0, img_object.rows);
		std::vector<Point2f> scene_corners(4);

		perspectiveTransform(obj_corners, scene_corners, H);

		//-- Desenha as linhas entre as quinas
		line(img_matches, scene_corners[0] + Point2f(img_object.cols, 0), scene_corners[1] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
		line(img_matches, scene_corners[1] + Point2f(img_object.cols, 0), scene_corners[2] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
		line(img_matches, scene_corners[2] + Point2f(img_object.cols, 0), scene_corners[3] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
		line(img_matches, scene_corners[3] + Point2f(img_object.cols, 0), scene_corners[0] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);

		//-- Exibe os matches
		imshow("Good Matches & Object detection", img_matches);

		waitKey(2);
	}
}

// Carrega a iluminacao e o objeto
void mydisplay(){

	glColor3f(1.0, 1.0, 1.0);
	GLfloat luzAmbiente0[4] = { 1.0, 0.0, 0.0, 1.0 }; // luz vermelha
	GLfloat luzDifusa[4] = { 0.7, 0.7, 0.7, 1.0 };
	GLfloat luzEspecular[4] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat posicaoLuz0[4] = { -5.0, 0.0, 0.0, 1.0 };

	glLoadIdentity();

	glEnable(GL_LIGHTING);
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, luzAmbiente0);

	// Define os parâmetros da luz de número 0 - vermelha
	glLightfv(GL_LIGHT0, GL_AMBIENT, luzAmbiente0);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, luzDifusa);
	glLightfv(GL_LIGHT0, GL_SPECULAR, luzEspecular);
	glLightfv(GL_LIGHT0, GL_POSITION, posicaoLuz0);

	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, luzAmbiente0);
	glEnable(GL_LIGHT0);

	GLfloat diff[] = { 1.0, 1.0, 1.0 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, luzEspecular);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	gluLookAt(0, 20, 30, 0, 0, 0, 0, 1, 0);


	obj.Draw();


	glutSwapBuffers();

	surf();

}

// Função usada para especificar o volume de visualização
void EspecificaParametrosVisualizacao(void)
{
	// Especifica sistema de coordenadas de projeção
	glMatrixMode(GL_PROJECTION);

	// Inicializa sistema de coordenadas de projeção
	glLoadIdentity();

	// Especifica a projeção perspectiva
	gluPerspective(angle, fAspect, 0.4, 500);

	// Especifica sistema de coordenadas do modelo
	glMatrixMode(GL_MODELVIEW);

	// Inicializa sistema de coordenadas do modelo
	glLoadIdentity();

	// Especifica posição do observador e do alvo
	gluLookAt(0, 20, 30, 0, 0, 0, 0, 1, 0);
}


void myreshape(GLsizei w, GLsizei h)
{
	// Para previnir uma divisão por zero
	if (h == 0) h = 1;

	// Especifica o tamanho da viewport
	glViewport(0, 0, w, h);

	// Calcula a correção de aspecto
	fAspect = (GLfloat)w / (GLfloat)h;

	EspecificaParametrosVisualizacao();
}


void initialize()
{
	glMatrixMode(GL_PROJECTION);
	GLfloat aspect = (GLfloat)win.width / win.height;
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(win.field_of_view_angle, aspect, win.z_near, win.z_far);
	glMatrixMode(GL_MODELVIEW);
	glShadeModel(GL_SMOOTH);
	glEnable(GL_NORMALIZE);
	glClearColor(0.0f, 0.1f, 0.0f, 0.5f);
	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

	// Capacidade de brilho do material
	GLfloat especularidade[4] = { 1.0, 1.0, 1.0, 1.0 };
	GLint especMaterial = 60;

	// Especifica que a cor de fundo da janela será preta
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

	// Habilita o modelo de colorização de Gouraud
	glShadeModel(GL_SMOOTH);

	// Define a refletância do material 
	glMaterialfv(GL_FRONT, GL_SPECULAR, especularidade);
	// Define a concentração do brilho
	glMateriali(GL_FRONT, GL_SHININESS, especMaterial);

	// Habilita a definição da cor do material a partir da cor corrente
	glEnable(GL_COLOR_MATERIAL);
	//Habilita o uso de iluminação
	glEnable(GL_LIGHTING);

	// Habilita o depth-buffering
	glEnable(GL_DEPTH_TEST);

	glutSolidTeapot(5.0);

	angle = 45;
}

int main(int argc, char** argv)
{

	cap.open("Resources\\InputData\\video.MOV");
	if (!cap.isOpened())
	{
		cout << "error, could not open the capture" << endl;
		system("pause");
		exit(1);
	}

	img_object = imread("Resources\\InputData\\livro.jpg", 1);

	Size size(200, 300);
	resize(img_object, img_object, size);

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glViewport(0, 200, 400, 200);
	glutInitWindowSize(800, 600);
	glutInitWindowPosition(0, 400);
	glutCreateWindow("Projeto 2");
	glutDisplayFunc(mydisplay);
	glutReshapeFunc(myreshape);
	initialize();
	obj.Load("dog.obj");
	glutMainLoop();

	return 0;
}









// TEREMOS QUE CALCULAR A POSE USANDO solvePnP - pontos do obj e pontos da imagem

// Código para desenhar uma imagem no background - codigo enviado por email por Marcel
// Os comentarios sao trechos de codigo que precisam ser corrigidos
void drawBackground()
{
	glDisable(GL_DEPTH_TEST);

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	//glGetFloatv(GL_MODELVIEW_MATRIX, modelview);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	//glGetFloatv(GL_PROJECTION_MATRIX, projection);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	glOrtho(0, win.width, win.height, 0, -1.0, 1.0);

	glRasterPos2d(0, win.height);
	cv::Mat resized;
	//cv::flip(color, resized, 0);
	glDrawPixels(resized.cols, resized.rows, GL_BGR_EXT, GL_UNSIGNED_BYTE, resized.data);

	glViewport(0, 0, win.width, win.height);
	glMatrixMode(GL_PROJECTION);
	//glLoadMatrixf(projection);
	glMatrixMode(GL_MODELVIEW);
	//glLoadMatrixf(modelview);
}