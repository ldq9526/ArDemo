#pragma once
#include <opencv2\opencv.hpp>
class ModelPly
{
public:
	/* vertex number */
	int nv;

	/* face number */
	int nf;

	/* transparentation */
	double alpha;

	/* set of vertex , N x 3,double */
	cv::Mat vertex;

	/* set of face */
	std::vector<cv::Vec3i> face;

	/* set of face normal */
	std::vector<cv::Vec3d> normal;

	/* ambient albedo of BGR */
	cv::Vec3d ambientAlbedo;

	/* diffuse albedo of BGR */
	cv::Vec3d diffuseAlbedo;

	/* specular albedo of BGR */
	cv::Vec3d specularAlbedo;

	ModelPly();
	ModelPly(cv::String fileName, double al, cv::Vec3d a, cv::Vec3d d, cv::Vec3d s);
	ModelPly(cv::String fileName, double al, double a[3], double d[3], double s[3]);

	/* compute surface normal of each face */
	void computeSurfaceNormal(void);

	/* test if the point X is on the face index */
	bool onFace(cv::Vec3d X, int index);

	~ModelPly();
};

