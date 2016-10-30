#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/video/video.hpp>

#include <iostream>

#include "../bgfg_KNN.h"

int main(){
	cv::VideoCapture cap(0);

	if(!cap.isOpened()){
		std::cout << " failed to initialize camera " << std::endl;
	}

	//cv::Ptr<cv::BackgroundSubtractor> fgbg = cv::createBackgroundSubtractorKNN(5,5,false);
	cv::BackgroundSubtractorKNN fgbg(500,400.0,true);

	//cv::Ptr<cv::BackgroundSubtractor> fgbg = 
	//	cv::Ptr<cv::BackgroundSubtractor>(new cv::BackgroundSubtractorMOG2(5,5));

	//cv::Ptr<cv::BackgroundSubtractor> fgbg = 
	//	cv::Ptr<cv::BackgroundSubtractor>(new cv::BackgroundSubtractorKNNImpl(5,5,false));

	cv::Mat frame;
	cv::Mat fgmask;

	cv::namedWindow("frame");
	cv::namedWindow("mask");

	while(1){
		if(!cap.read(frame))
			break;
		
		fgbg.apply(frame,fgmask,0.5);

		try{
			imshow("frame", frame);
			imshow("mask", fgmask);
		}
		catch(...){
			std::cerr << "EXCEPTION! " << std::endl;
		}
		char k = cv::waitKey(20);
		if(k == 27)
			break;
	}

	cap.release();

	cv::destroyAllWindows();

	return 0;

}
