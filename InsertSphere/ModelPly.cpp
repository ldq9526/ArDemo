#include "ModelPly.h"


ModelPly::ModelPly()
{
}

ModelPly::ModelPly(cv::String fileName, double al, cv::Vec3d a, cv::Vec3d d, cv::Vec3d s)
{
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
	FILE * fp;
	fopen_s(&fp, fileName.c_str(), "r");
	if (fp == NULL)
	{
		std::cout << "No model file :" << fileName << std::endl;
		return;
	}
	fscanf_s(fp, "element vertex %d\n", &nv);
	fscanf_s(fp, "element face %d\n", &nf);
	vertex = cv::Mat(nv, 3, CV_64FC1);
	for (int i = 0; i < nv; i++)
	{
		double x, y, z;
		fscanf_s(fp, "%lf %lf %lf %*lf %*lf \n", &x, &y, &z);
		vertex.at<double>(i, 0) = x;
		vertex.at<double>(i, 1) = y;
		vertex.at<double>(i, 2) = z;
	}
	for (int i = 0; i < nf; i++)
	{
		int x, y, z;
		fscanf_s(fp, "3 %d %d %d \n", &x, &y, &z);
		cv::Vec3i vf(x, y, z);
		face.push_back(vf);
	}
	fclose(fp);
	computeSurfaceNormal();
	std::cout << "Finish loading ply model !" << std::endl;
}

ModelPly::ModelPly(cv::String fileName, double al, double a[3], double d[3], double s[3])
{
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
	FILE * fp;
	fopen_s(&fp, fileName.c_str(), "r");
	if (fp == NULL)
	{
		std::cout << "No model file :" << fileName << std::endl;
		return;
	}
	fscanf_s(fp, "element vertex %d\n", &nv);
	fscanf_s(fp, "element face %d\n", &nf);
	vertex = cv::Mat(nv, 3, CV_64FC1);
	for (int i = 0; i < nv; i++)
	{
		double x, y, z;
		fscanf_s(fp, "%lf %lf %lf %*lf %*lf \n", &x, &y, &z);
		vertex.at<double>(i, 0) = x;
		vertex.at<double>(i, 1) = y;
		vertex.at<double>(i, 2) = z;
	}
	for (int i = 0; i < nf; i++)
	{
		int x, y, z;
		fscanf_s(fp, "3 %d %d %d \n", &x, &y, &z);
		cv::Vec3i vf(x, y, z);
		face.push_back(vf);
	}
	fclose(fp);
	computeSurfaceNormal();
	std::cout << "Finish loading ply model !" << std::endl;
}

/* compute surface normal of each face */
void ModelPly::computeSurfaceNormal(void)
{
	for (int i = 0; i < nf; i++)
	{
		cv::Vec3d a(vertex.at<double>(face[i][0],0), vertex.at<double>(face[i][0],1), vertex.at<double>(face[i][0],2));
		cv::Vec3d b(vertex.at<double>(face[i][1],0), vertex.at<double>(face[i][1],1), vertex.at<double>(face[i][1],2));
		cv::Vec3d c(vertex.at<double>(face[i][2],0), vertex.at<double>(face[i][2],1), vertex.at<double>(face[i][2],2));
		cv::Vec3d x = a - b, y = a - c;
		cv::Vec3d n = x.cross(y);
		if (a.dot(n) < 0)
			n = -n;
		n /= sqrt(n.dot(n));
		normal.push_back(n);
	}
}

/* test if the point X is on the face index */
bool ModelPly::onFace(cv::Vec3d X, int index)
{
	cv::Vec3d a(vertex.at<double>(face[index][0], 0), vertex.at<double>(face[index][0], 1), vertex.at<double>(face[index][0], 2));
	cv::Vec3d b(vertex.at<double>(face[index][1], 0), vertex.at<double>(face[index][1], 1), vertex.at<double>(face[index][1], 2));
	cv::Vec3d c(vertex.at<double>(face[index][2], 0), vertex.at<double>(face[index][2], 1), vertex.at<double>(face[index][2], 2));
	cv::Vec3d x = b - a, y = c - a, p = X - a;
	double u = (p.dot(x)*y.dot(y) - p.dot(y)*x.dot(y)) / (x.dot(x)*y.dot(y) - (x.dot(y))*(x.dot(y)));
	double v = (p.dot(y)*x.dot(x) - p.dot(x)*x.dot(y)) / (x.dot(x)*y.dot(y) - (x.dot(y))*(x.dot(y)));
	return u >= 0 && v >= 0 && u + v <= 1;
}


ModelPly::~ModelPly()
{
}
