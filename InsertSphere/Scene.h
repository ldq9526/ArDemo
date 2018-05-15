#pragma once
#include <opencv2\opencv.hpp>
#include "ModelSphere.h"
#include "ModelPly.h"

/* type of the data */
const uint TYPE_KINECT = 1;

class Camera
{
public:
	double ux, uy, f;
};

class LightSource
{
public:
	cv::Vec3d position3D;/* position of the light source */
	cv::Vec3d intensity;/* intensity of the RGB light source */
};

class Scene
{
private:

	/* type of the data */
	uint type;

	/* BGR image of the scene */
	cv::Mat image;

	/* depth map */
	cv::Mat depthMap;

	/* normalized surface normal of each pixel */
	cv::Mat surfaceNormal;

	/* 3D position of each pixel */
	cv::Mat position3D;

	/* light source */
	LightSource lightSource;

	/* camera model */
	Camera camera;

	/* compute depth map */
	void computeDepthMap(void);

	/* compute only 3D position of each pixel */
	void computePosition3D(void);

	/* compute only surface normal of each pixel */
	void computeSurfaceNormal(void);

	/* compute both 3D position and surface normal of each pixel */
	void computeGeometry(void);

	/* compute camera model */
	void computeCamera(void);

	/* compute light source */
	void computeLightSource(void);

	/* fit a plane normal using 5 pixels around point(x,y) */
	cv::Vec3d fitPlane(int row, int col);

	/* fit a plane normal using 5 pixels around point(x,y) */
	cv::Vec3d fitPlane(cv::Point p);
public:
	Scene();
	Scene(cv::String imageName);
	Scene(cv::String imageName, cv::String depthName, uint type);
	Scene(cv::Mat image);

	/* get original image of the scene */
	cv::Mat getImage(void);

	/* get the original depth map of the scene */
	cv::Mat getDepthMap(void);

	/* get the visual depth mapth */
	cv::Mat getVisualDepthMap(void);

	/* get the normalized surface normal */
	cv::Mat getSurfaceNormal(void);

	/* get the visual surface normal */
	cv::Mat getVisualSurfaceNormal(void);

	/* get the 3D position of each pixel */
	cv::Mat getPosition3D(void);

	/* get the camera model */
	Camera getCamera(void);

	/* get the light source */
	LightSource getLightSource(void);

	/* insert a sphere model into the scene */
	cv::Mat insertModel(int row,int col,ModelSphere m);

	/* insert a sphere model into the scene */
	cv::Mat insertModel(cv::Point p, ModelSphere m);

	/* insert a sphere model into the scene */
	cv::Mat insertModel(int row, int col, ModelPly m);

	/* insert a sphere model into the scene */
	cv::Mat insertModel(cv::Point p, ModelPly m);

	~Scene();
};

