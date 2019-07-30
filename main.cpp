#include <iostream>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core.hpp"
#include <chrono>
#include <unistd.h>

using namespace cv;
using namespace std;

Mat gray;
Mat blue;
Mat green;
Mat red;
Mat yellow;
Mat black;

Mat cameraFrame;

Mat channels[3];

cv::TickMeter tm;

double fps1 = 0;
double fps2 = 0;
double fps3 = 0;
double totalFps = 0;
long totalFpsCount = 1;

/// Global Variables
int alpha_slider;
double alpha;

Mat correctGamma( Mat& img, double gamma ) {
 double inverse_gamma = 1.0 / gamma;
 
 Mat lut_matrix(1, 256, CV_8UC1 );
 uchar * ptr = lut_matrix.ptr();
 for( int i = 0; i < 256; i++ )
   ptr[i] = (int)( pow( (double) i / 255.0, inverse_gamma ) * 255.0 );
 
 Mat result;
 LUT( img, lut_matrix, result );
 
 return result;
}

int main(int argc, const char * argv[]) {
    //Capture stream from webcam.
    VideoCapture capture("rkcamsrc io-mode=4 isp-mode=2A ! video/x-raw,format=NV12,width=640,height=480 ! videoconvert ! appsink");

    //Check if we can get the webcam stream.
    if(!capture.isOpened()) {
        cout << "Could not open camera" << endl;
        return -1;
    }
	//long frameCounter = 0;
	//std::time_t timeBegin = std::time(0);
	//int tick = 0;
    
	// Create a window
    namedWindow("Original Image", 1);

    //Create trackbar to change brightness
    int gammaSlider = 100;
    createTrackbar("Gamma", "Original Image", &gammaSlider, 500);
    int imageShownSlider = 1;
    createTrackbar("Preview Images", "Original Image", &imageShownSlider, 1);
    int tr = 80;
    createTrackbar("Red Treshold multiplier", "Original Image", &tr, 100);
    int tb = 80;
    createTrackbar("Blue Treshold multiplier", "Original Image", &tb, 100);
    int ty = 80;
    createTrackbar("Yellow Treshold multiplier", "Original Image", &ty, 100);
	capture.read(cameraFrame);
	capture.read(cameraFrame);
	capture.read(cameraFrame);
	capture.read(cameraFrame);
	capture.read(cameraFrame);
	
	Mat channelsc[3];
	Mat yellowc;
	extractChannel(cameraFrame, channelsc[0], 0);
	extractChannel(cameraFrame, channelsc[1], 1);
	extractChannel(cameraFrame, channelsc[2], 2);
	Mat bluec = channelsc[0];
	Mat greenc = channelsc[1];
	Mat redc = channelsc[2];
	
	Mat grayc;
	cvtColor(cameraFrame, grayc, COLOR_BGR2GRAY);
	subtract(grayc, bluec, yellowc);
	subtract(bluec, grayc, bluec);
	subtract(redc, grayc, redc);
	subtract(yellowc, redc, yellowc);
	subtract(redc, yellowc, redc);
	
	double minr, maxr, minb, maxb, miny, maxy;
	minMaxLoc(redc, &minr, &maxr, NULL, NULL);
	minMaxLoc(bluec, &minb, &maxb, NULL, NULL);
	minMaxLoc(yellowc, &miny, &maxy, NULL, NULL);
	while (true) {
		auto start = chrono::steady_clock::now();
        	//Read an image from the camera.
        	capture >> cameraFrame;
        	auto ReadFrame = chrono::steady_clock::now();
        	long deltaReadFrame = chrono::duration_cast<chrono::milliseconds>(ReadFrame - start).count();
		cout << "deltaReadFrame: " << deltaReadFrame << endl;
		
		double gamma = gammaSlider / 100.0;
		cout << "Gamma: " << gamma << endl;
		cameraFrame = correctGamma(cameraFrame, gamma);
		auto GammaTime = chrono::steady_clock::now();
        	long deltaGamma = chrono::duration_cast<chrono::milliseconds>(GammaTime - ReadFrame).count();
		cout << "deltaGamma: " << deltaGamma << endl;
		
		blur(cameraFrame, cameraFrame, Size(10, 10));
		auto Blur = chrono::steady_clock::now();
        	long deltaBlur = chrono::duration_cast<chrono::milliseconds>(Blur - GammaTime).count();
		cout << "deltaBlur: " << deltaBlur << endl;
		
		split(cameraFrame, channels);
		blue = channels[0];
		green = channels[1];
		red = channels[2];

		
		cvtColor(cameraFrame, gray, COLOR_BGR2GRAY);
		subtract(gray, blue, yellow);
		subtract(blue, gray, blue);
		subtract(red, gray, red);
		subtract(yellow, red, yellow);
		subtract(red, yellow, red);
		subtract(green, gray, green);
		auto Normalization = chrono::steady_clock::now();
        	long deltaNormalization = chrono::duration_cast<chrono::milliseconds>(Normalization - Blur).count();
		cout << "deltaNormalization: " << deltaNormalization << endl;
		
			/*channels[0] = blue * 10;
			channels[1] = green * 25;
			channels[2] = red * 2;
			
			Mat seg_img;
			merge(channels, 3, seg_img);*/
		
		
		
		threshold(red, red, maxr * tr / 100, 255, THRESH_BINARY);
		threshold(blue, blue, maxb * tb / 100, 255, THRESH_BINARY);
		threshold(yellow, yellow, maxy * ty / 100, 255, THRESH_BINARY);
		auto Threshold = chrono::steady_clock::now();
        	long deltaThreshold = chrono::duration_cast<chrono::milliseconds>(Threshold - Normalization).count();
		cout << "deltaThreshold: " << deltaThreshold << endl;
		
		
		if (imageShownSlider == 1) {
    			imshow("Original Image", cameraFrame);
    			//imshow("Segmented Image", seg_img);
			imshow("Gray", gray);
			imshow("Ball", red);
			imshow("Blue Goal", blue);
			imshow("Yellow Goal", yellow);
		}
		totalFps += fps3;
		fps1 = fps2;
		fps2 = fps3;
		auto end = chrono::steady_clock::now();
		fps3 = chrono::duration_cast<chrono::milliseconds>(end - start).count();
		fps3 = 1000 / fps3;
		double avg3 = (fps1 + fps2 + fps3) / 3;
		double totalavg = totalFps / totalFpsCount;
		cout << "Current FPS: " << fps3 << endl;
		cout << "Running Average (3) FPS: " << avg3 << endl;
		cout << "Total Average FPS: " << totalavg << endl;
		totalFpsCount++;
		char key = (char) waitKey(1);
        if (key == 'q' || key == 27)
        {
            break;
        }
	}
	return 0;
}