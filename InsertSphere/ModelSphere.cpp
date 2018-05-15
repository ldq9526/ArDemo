#include "ModelSphere.h"


ModelSphere::ModelSphere()
{
}

ModelSphere::ModelSphere(double r, double al ,cv::Vec3d a, cv::Vec3d d, cv::Vec3d s)
{
	radius = r;
	alpha = al;
	ambientAlbedo[0] = a[0];
	ambientAlbedo[1] = a[1];
	ambientAlbedo[2] = a[2];
	diffuseAlbedo[0] = d[0];
	diffuseAlbedo[1] = d[1];
	diffuseAlbedo[2] = d[2];
	specularAlbedo[0] = s[0];
	specularAlbedo[1] = s[1];
	specularAlbedo[2] = s[2];
}

ModelSphere::ModelSphere(double r, double al, double a[3], double d[3], double s[3])
{
	radius = r;
	alpha = al;
	ambientAlbedo[0] = a[0];
	ambientAlbedo[1] = a[1];
	ambientAlbedo[2] = a[2];
	diffuseAlbedo[0] = d[0];
	diffuseAlbedo[1] = d[1];
	diffuseAlbedo[2] = d[2];
	specularAlbedo[0] = s[0];
	specularAlbedo[1] = s[1];
	specularAlbedo[2] = s[2];
}


ModelSphere::~ModelSphere()
{
}
