#include "Scene.h"
#include <cmath>


Scene::Scene()
{
}

Scene::Scene(cv::String imageName)
{

}

Scene::Scene(cv::String imageName, cv::String depthName, uint type)
{
	std::cout << "Loading the scene ..." << std::endl;
	image = cv::imread(imageName, cv::IMREAD_UNCHANGED);
	if (!image.data)
	{
		std::cout << "No image file :" << imageName << "!" << std::endl;
		return;
	}
	depthMap = cv::imread(depthName, cv::IMREAD_UNCHANGED);
	if (!depthMap.data)
	{
		std::cout << "No depth map image file :" << depthName << "!" << std::endl;
		return;
	}
	this->type = type;
	surfaceNormal = cv::Mat(image.size(), CV_64FC3);
	position3D = cv::Mat(image.size(), CV_64FC3);
	
	computeCamera();
	computeDepthMap();
	computeGeometry();
	computeLightSource();
	std::cout << "Finished loading !" << std::endl;
}



Scene::Scene(cv::Mat image)
{

}


Scene::~Scene()
{
}

/* compute depth map */
void Scene::computeDepthMap(void)
{
	if (type == TYPE_KINECT)
		depthMap.convertTo(depthMap, CV_64FC1, 1.0 / 1000);
	else
		depthMap.convertTo(depthMap, CV_64FC1, 1.0);
}

/* compute 3D position of each pixel */
void Scene::computePosition3D(void)
{
	double cx = camera.ux, cy = camera.uy, f = camera.f;
	cv::Mat & D = depthMap;
	cv::Mat & P = position3D;
	for (int i = 0; i < image.size().width; i++)
	{
		for (int j = 0; j < image.size().height; j++)
		{
			P.at<cv::Vec3d>(j, i)[0] = D.at<double>(j, i)*(double(i) - cx) / f;
			P.at<cv::Vec3d>(j, i)[1] = D.at<double>(j, i)*(double(j) - cy) / f;
			P.at<cv::Vec3d>(j, i)[2] = D.at<double>(j, i);
		}
	}
}

/* compute surface normal of each pixel */
void Scene::computeSurfaceNormal(void)
{
	double cx = camera.ux, cy = camera.uy, f = camera.f;
	cv::Mat & N = surfaceNormal;
	cv::Mat & D = depthMap;
	for (int i = 0; i < image.size().width; i++)
	{
		for (int j = 0; j < image.size().height; j++)
		{
			if (i == 0 && j == 0)
			{
				N.at<cv::Vec3d>(j, i)[0] = D.at<double>(j + 1, i)*(D.at<double>(j, i + 1) - D.at<double>(j, i)) / f;
				N.at<cv::Vec3d>(j, i)[1] = D.at<double>(j, i + 1)*(D.at<double>(j + 1, i) - D.at<double>(j, i)) / f;
				N.at<cv::Vec3d>(j, i)[2] = N.at<cv::Vec3d>(j, i)[0] * (cx - double(i)) / f + N.at<cv::Vec3d>(j, i)[1] * (cy - double(j)) / f - D.at<double>(j, i + 1)*D.at<double>(j + 1, i) / f / f;
			}
			else if (i > 0 && j == 0)
			{
				N.at<cv::Vec3d>(j, i)[0] = D.at<double>(j + 1, i)*(D.at<double>(j, i) - D.at<double>(j, i - 1)) / f;
				N.at<cv::Vec3d>(j, i)[1] = D.at<double>(j, i - 1)*(D.at<double>(j + 1, i) - D.at<double>(j, i)) / f;
				N.at<cv::Vec3d>(j, i)[2] = N.at<cv::Vec3d>(j, i)[0] * (cx - double(i)) / f + N.at<cv::Vec3d>(j, i)[1] * (cy - double(j)) / f - D.at<double>(j, i - 1)*D.at<double>(j + 1, i) / f / f;
			}
			else if (i == 0 && j > 0)
			{
				N.at<cv::Vec3d>(j, i)[0] = D.at<double>(j - 1, i)*(D.at<double>(j, i + 1) - D.at<double>(j, i)) / f;
				N.at<cv::Vec3d>(j, i)[1] = D.at<double>(j, i + 1)*(D.at<double>(j, i) - D.at<double>(j - 1, i)) / f;
				N.at<cv::Vec3d>(j, i)[2] = N.at<cv::Vec3d>(j, i)[0] * (cx - double(i)) / f + N.at<cv::Vec3d>(j, i)[1] * (cy - double(j)) / f - D.at<double>(j, i + 1)*D.at<double>(j - 1, i) / f / f;
			}
			else
			{
				N.at<cv::Vec3d>(j, i)[0] = D.at<double>(j - 1, i)*(D.at<double>(j, i) - D.at<double>(j, i - 1)) / f;
				N.at<cv::Vec3d>(j, i)[1] = D.at<double>(j, i - 1)*(D.at<double>(j, i) - D.at<double>(j - 1, i)) / f;
				N.at<cv::Vec3d>(j, i)[2] = N.at<cv::Vec3d>(j, i)[0] * (cx - double(i)) / f + N.at<cv::Vec3d>(j, i)[1] * (cy - double(j)) / f - D.at<double>(j, i - 1)*D.at<double>(j - 1, i) / f / f;
			}
			double len = sqrt(N.at<cv::Vec3d>(j, i).dot(N.at<cv::Vec3d>(j, i)));
			if (len < 1e-5)
				N.at<cv::Vec3d>(j, i) *= 0;
			else
				N.at<cv::Vec3d>(j, i) /= len;
		}
	}
}

/* compute 3D position of each pixel */
void Scene::computeGeometry(void)
{
	double cx = camera.ux, cy = camera.uy, f = camera.f;
	cv::Mat & N = surfaceNormal;
	cv::Mat & D = depthMap;
	cv::Mat & P = position3D;
	for (int i = 0; i < image.size().width; i++)
	{
		for (int j = 0; j < image.size().height; j++)
		{
			P.at<cv::Vec3d>(j, i)[0] = D.at<double>(j, i)*(double(i) - cx) / f;
			P.at<cv::Vec3d>(j, i)[1] = D.at<double>(j, i)*(double(j) - cy) / f;
			P.at<cv::Vec3d>(j, i)[2] = D.at<double>(j, i);
			if (i == 0 && j == 0)
			{
				N.at<cv::Vec3d>(j, i)[0] = D.at<double>(j + 1, i)*(D.at<double>(j, i + 1) - D.at<double>(j, i)) / f;
				N.at<cv::Vec3d>(j, i)[1] = D.at<double>(j, i + 1)*(D.at<double>(j + 1, i) - D.at<double>(j, i)) / f;
				N.at<cv::Vec3d>(j, i)[2] = N.at<cv::Vec3d>(j, i)[0] * (cx - double(i)) / f + N.at<cv::Vec3d>(j, i)[1] * (cy - double(j)) / f - D.at<double>(j, i + 1)*D.at<double>(j + 1, i) / f / f;
			}
			else if (i > 0 && j == 0)
			{
				N.at<cv::Vec3d>(j, i)[0] = D.at<double>(j + 1, i)*(D.at<double>(j, i) - D.at<double>(j, i - 1)) / f;
				N.at<cv::Vec3d>(j, i)[1] = D.at<double>(j, i - 1)*(D.at<double>(j + 1, i) - D.at<double>(j, i)) / f;
				N.at<cv::Vec3d>(j, i)[2] = N.at<cv::Vec3d>(j, i)[0] * (cx - double(i)) / f + N.at<cv::Vec3d>(j, i)[1] * (cy - double(j)) / f - D.at<double>(j, i - 1)*D.at<double>(j + 1, i) / f / f;
			}
			else if (i == 0 && j > 0)
			{
				N.at<cv::Vec3d>(j, i)[0] = D.at<double>(j - 1, i)*(D.at<double>(j, i + 1) - D.at<double>(j, i)) / f;
				N.at<cv::Vec3d>(j, i)[1] = D.at<double>(j, i + 1)*(D.at<double>(j, i) - D.at<double>(j - 1, i)) / f;
				N.at<cv::Vec3d>(j, i)[2] = N.at<cv::Vec3d>(j, i)[0] * (cx - double(i)) / f + N.at<cv::Vec3d>(j, i)[1] * (cy - double(j)) / f - D.at<double>(j, i + 1)*D.at<double>(j - 1, i) / f / f;
			}
			else
			{
				N.at<cv::Vec3d>(j, i)[0] = D.at<double>(j - 1, i)*(D.at<double>(j, i) - D.at<double>(j, i - 1)) / f;
				N.at<cv::Vec3d>(j, i)[1] = D.at<double>(j, i - 1)*(D.at<double>(j, i) - D.at<double>(j - 1, i)) / f;
				N.at<cv::Vec3d>(j, i)[2] = N.at<cv::Vec3d>(j, i)[0] * (cx - double(i)) / f + N.at<cv::Vec3d>(j, i)[1] * (cy - double(j)) / f - D.at<double>(j, i - 1)*D.at<double>(j - 1, i) / f / f;
			}
			double len = sqrt(N.at<cv::Vec3d>(j, i).dot(N.at<cv::Vec3d>(j, i)));
			if (len < 1e-5)
				N.at<cv::Vec3d>(j, i) *= 0;
			else
				N.at<cv::Vec3d>(j, i) /= len;
		}
	}
}

/* compute camera model */
void Scene::computeCamera(void)
{
	camera.ux = 319.5;
	camera.uy = 239.5;
	camera.f = 525.0;
}

/* compute light source */
void Scene::computeLightSource(void)
{
	lightSource.position3D[0] = -1.5;
	lightSource.position3D[1] = -1.0;
	lightSource.position3D[2] = -0.5;
	lightSource.intensity[0] = 0.8;
	lightSource.intensity[1] = 0.8;
	lightSource.intensity[2] = 0.8;
}

/* get original image of the scene */
cv::Mat Scene::getImage(void)
{
	cv::Mat tmp;
	image.copyTo(tmp);
	return tmp;
}

/* get the original depth map of the scene */
cv::Mat Scene::getDepthMap(void)
{
	cv::Mat tmp;
	depthMap.copyTo(tmp);
	return tmp;
}

/* get the visual depth mapth */
cv::Mat Scene::getVisualDepthMap(void)
{
	std::vector<cv::Mat> vdepth;
	cv::Mat nd;
	cv::normalize(depthMap, nd, 1.0, 0.0, cv::NORM_MINMAX);
	vdepth.push_back(nd);
	vdepth.push_back(nd);
	vdepth.push_back(nd);
	cv::Mat vd;
	cv::merge(vdepth, vd);
	return vd;
}

/* get the normalized surface normal */
cv::Mat Scene::getSurfaceNormal(void)
{
	cv::Mat tmp;
	surfaceNormal.copyTo(tmp);
	return tmp;
}

/* get the visual surface normal */
cv::Mat Scene::getVisualSurfaceNormal(void)
{
	return surfaceNormal*0.5 + 0.5;
}

/* get the 3D position of each pixel */
cv::Mat Scene::getPosition3D(void)
{
	cv::Mat tmp;
	position3D.copyTo(tmp);
	return tmp;
}

/* fit a plane normal using 5 pixels around point(x,y) */
cv::Vec3d Scene::fitPlane(cv::Point p)
{
	int r = 3;
	cv::Mat & P = position3D;
	double A = 0, B = 0, C = 0, D = 0, E = 0, F = 0, G = 0, H = 0, I = 0;
	for (int i = p.y - r; i <= p.y + r; i++)
	{
		for (int j = p.x - r; j <= p.x + r; j++)
		{
			if (i >= 0 && i < image.size().height && j >= 0 && j <image.size().width)
			{
				double x = P.at<cv::Vec3d>(i, j)[0], y = P.at<cv::Vec3d>(i, j)[1], z = P.at<cv::Vec3d>(i, j)[2];
				A += x;
				B += y;
				C += z;
				D += x*x;
				E += y*y;
				F += z*z;
				G += x*y;
				H += x*z;
				I += y*z;
			}
		}
	}
	double a[3][3] = { { D, G, H }, { G, E, I }, { H, I, F } };
	double b[3][1] = { { A }, { B }, { C } };
	cv::Mat AL(3, 3, CV_64FC1, (double *)a);
	cv::Mat BL(3, 1, CV_64FC1, (double *)b);
	cv::Mat N = AL.inv() * BL;
	N = N / sqrt(N.dot(N));
	if (N.at<double>(1, 0) > 0)
		N = -N;
	return cv::Vec3d(N.at<double>(0, 0), N.at<double>(1, 0), N.at<double>(2, 0));
}

/* fit a plane normal using 5 pixels around point(x,y) */
cv::Vec3d Scene::fitPlane(int row, int col)
{
	cv::Point p(col, row);
	return fitPlane(p);
}

/* insert a sphere model into the scene */
cv::Mat Scene::insertModel(cv::Point p, ModelSphere m)
{
	/* parameters of the camera */
	double ux = camera.ux, uy = camera.uy, f = camera.f;

	/* radius of the sphere in the world coordinate */
	double R = m.radius;
	
	/* surface normal of the plane to be inserted */
	cv::Vec3d N = fitPlane(p);

	/* world coordinate of the point on the ground */
	cv::Vec3d P = position3D.at<cv::Vec3d>(p);

	/* world coordinate of the sphere center */
	cv::Vec3d SC = P +N*R;

	/* light position */
	cv::Vec3d & LP = lightSource.position3D;

	/* light intensity */
	cv::Vec3d & LI = lightSource.intensity;

	/* sphere center on image plane */
	cv::Vec2d sc(ux+SC[0]*f/SC[2],uy+SC[1]*f/SC[2]);

	/* radius on image plane */
	double r = f*R / SC[2];

	/* get the image normalized to [0,1] */
	cv::Mat I,nI;
	image.convertTo(I, CV_64FC3, 1.0 / 255);
	I.copyTo(nI);

	for (double i = sc[0] - r; i <= sc[0] + r; i += 0.1)
		for (double j = sc[1] - r; j <= sc[1] + r; j += 0.1)
		{
			if (sqrt((i - sc[0])*(i - sc[0]) + (j - sc[1])*(j - sc[1])) <= r)
			{
				cv::Vec3d fac((i - ux) / f, (j - uy) / f, 1);
				double A = fac.dot(fac);
				double B = -2 * fac.dot(SC);
				double C = SC.dot(SC) - R*R;
				double delta = B*B - 4 * A*C;
				if (delta >= 0)
				{
					/* compute possible depth */
					double d[2] = { (-B + sqrt(delta)) / 2 / A, (-B - sqrt(delta)) / 2 / A };
					for (int k = 0; k < 2; k++)
					{
						/* get the intersection point on the sphere surface */
						cv::Vec3d IP = fac * d[k];
						if (N.dot(LP - IP) != 0)
						{
							cv::Vec3d GP = IP + N.dot(P - IP) * (LP - IP) / N.dot(LP - IP);/* intersection on the ground */
							int x = int(ux + GP[0] * f / GP[2]), y = int(uy + GP[1] * f / GP[2]);
							if (x >= 0 && x < I.size().width && y >= 0 && y < I.size().height)
								nI.at<cv::Vec3d>(y, x) = 0.8 * I.at<cv::Vec3d>(y,x);
						}
					}
					cv::Vec3d IP = fac * d[1];
					cv::Vec3d n = (IP - SC) / R;/* surface normal on sphere */
					cv::Vec3d v = -IP / sqrt(IP.dot(IP));/* direction of view */
					cv::Vec3d l = LP - IP;
					l = l / sqrt(l.dot(l));/* direction from intersection to light */
					cv::Vec3d h = v + l;
					h = h / sqrt(h.dot(h));/* direction between v and l */
					int x = int(i), y = int(j);
					if (x >= 0 && x < I.size().width && j >= 0 && j < I.size().height)
					{
						cv::Vec3d bgr = m.ambientAlbedo + LI.mul(cv::max(0.0, n.dot(l))*m.diffuseAlbedo + pow(cv::max(0.0, n.dot(h)), 50)*m.specularAlbedo);
						nI.at<cv::Vec3d>(y, x) = bgr;
					}
				}
			}
		}
	return nI;
}

/* insert a sphere model into the scene */
cv::Mat Scene::insertModel(int row, int col, ModelSphere m)
{
	cv::Point p(col, row);
	return insertModel(p, m);
}

/* insert a sphere model into the scene */
cv::Mat Scene::insertModel(int row, int col, ModelPly m)
{
	cv::Point p(col, row);
	return insertModel(p, m);
}

/* insert a sphere model into the scene */
cv::Mat Scene::insertModel(cv::Point p, ModelPly m)
{
	double maxd,maxv,minv;
	ModelPly cm(m);
	cv::minMaxIdx(depthMap, NULL, &maxd);
	cv::minMaxIdx(cm.vertex, &minv, &maxv);
	double R = 0.5 / maxd;
	double scale = R  / cv::max(fabs(minv), maxv);
	cm.vertex *= scale;

	/* light position */
	cv::Vec3d & LP = lightSource.position3D;

	/* light intensity */
	cv::Vec3d & LI = lightSource.intensity;

	/* surface normal of the plane to be inserted */
	cv::Vec3d N = fitPlane(p);

	/* world coordinate of the point on the ground */
	cv::Vec3d P = position3D.at<cv::Vec3d>(p);

	/* world coordinate of the model center */
	cv::Vec3d MC = P + N * 2 * R;
	for (int i = 0; i < cm.vertex.size().height;i++)
	{
		cm.vertex.at<double>(i, 0) += MC[0];
		cm.vertex.at<double>(i, 1) += MC[1];
		cm.vertex.at<double>(i, 2) += MC[2];
	}

	
	/* get the image normalized to [0,1] */
	cv::Mat nI;
	image.convertTo(nI, CV_64FC3, 1.0 / 255);
	for (int i = 0; i < nI.size().width; i++)
		for (int j = 0; j < nI.size().height; j++)
		{
			std::vector<double> tv;/* vector of t */
			std::vector<cv::Vec3d> iv;/* vector of intersection */
			std::vector<int> fv;/* vector of face index */
			for (int k = 0; k < cm.nf; k++)
			{
				cv::Vec3d P0(cm.vertex.at<double>(cm.face[k][0], 0), cm.vertex.at<double>(cm.face[k][0], 1), cm.vertex.at<double>(cm.face[k][0], 2));
				cv::Vec3d N(cm.normal[k]);
				cv::Vec3d Xn(position3D.at<double>(j, i)), Xf(position3D.at<double>(j, i));
				Xn[2] = 0;
				double t = N.dot(Xn - P0) / N.dot(Xn - Xf);
				if (t <= 1.0)
				{
					cv::Vec3d X = Xn + t*(Xf - Xn);
					if (cm.onFace(X, k))
					{
						tv.push_back(t);
						iv.push_back(X);
						fv.push_back(k);
					}
				}
			}
			if (tv.size() > 0)
			{
				std::vector<double>::iterator min_it = std::min_element(std::begin(tv), std::end(tv));
				int min_index = std::distance(std::begin(tv), min_it);/* get the index of the intersection and face */
				cv::Vec3d IP = iv[min_index];
				cv::Vec3d n = cm.normal[fv[min_index]];/* surface normal on sphere */
				cv::Vec3d v = -IP / sqrt(IP.dot(IP));/* direction of view */
				cv::Vec3d l = LP - IP;
				l = l / sqrt(l.dot(l));/* direction from intersection to light */
				cv::Vec3d h = v + l;
				h = h / sqrt(h.dot(h));/* direction between v and l */
				if (i >= 0 && i < nI.size().width && j >= 0 && j < nI.size().height)
				{
					nI.at<cv::Vec3d>(j, i) = m.ambientAlbedo + LI.mul(cv::max(0.0, n.dot(l))*m.diffuseAlbedo + pow(cv::max(0.0, n.dot(h)), 50)*m.specularAlbedo);
				}
			}
		}
	return nI;
}