#pragma once
#include <opencv2\opencv.hpp>

class ModelSphere
{
public:
	/* radius */
	double radius;

	/* alpha */
	double alpha;

	/* ambient albedo of BGR */
	cv::Vec3d ambientAlbedo;

	/* diffuse albedo of BGR */
	cv::Vec3d diffuseAlbedo;

	/* specular albedo of BGR */
	cv::Vec3d specularAlbedo;

	ModelSphere();
	ModelSphere(double r, double al, double a[3], double d[3], double s[3]);
	ModelSphere(double r, double al, cv::Vec3d a, cv::Vec3d d, cv::Vec3d s);
	~ModelSphere();
};

