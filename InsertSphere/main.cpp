#include <opencv2\opencv.hpp>
#include <iostream>
#include <string>
#include "Scene.h"

int main(int argc, char * argv[])
{
	Scene scene;
	if (argc == 3)
		scene = Scene(argv[1], argv[2], TYPE_KINECT);
	else
		scene = Scene("image.png", "depth.png", TYPE_KINECT);
	
	double ambientAlbedo[3] = { 0.1, 0.2, 0.2 };
	double diffuseAlbedo[3] = { 0.3, 0.6, 0.6 };
	double specularAlbedo[3] = { 0.1, 0.1, 0.1 };
	ModelSphere model(0.2, 0.5, ambientAlbedo, diffuseAlbedo, specularAlbedo);
	cv::Point point(479, 359);
	cv::Mat nI = scene.insertModel(point, model);
	cv::imshow("Composite", nI);
	

	cv::waitKey(0);
	cv::destroyAllWindows();
	std::cout << "Press ENTER to quit ..." << std::endl;
	getchar();
	return 0;
}