#include <iostream>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core.hpp"
#include <chrono>
#include <unistd.h>

using namespace cv;
using namespace std;

UMat gray;
UMat blue;
UMat green;
UMat red;
UMat yellow;
UMat black;
UMat ryG;
UMat gyG;

Mat cameraFrame;

UMat channels[3];

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
	
	while (true) {
		auto start = chrono::steady_clock::now();
        //Read an image from the camera.
        capture.read(cameraFrame);
		
		double gamma = gammaSlider / 100.0;
		cout << "Gamma: " << gamma << endl;
		cameraFrame = correctGamma(cameraFrame, gamma);
		//cameraFrame = imread("/Users/matteoschmider/Desktop/Foto.png", IMREAD_COLOR);
		
		extractChannel(cameraFrame, channels[0], 0);
		extractChannel(cameraFrame, channels[1], 1);
		extractChannel(cameraFrame, channels[2], 2);
		blue = channels[0];
		green = channels[1];
		red = channels[2];
		
		cvtColor(cameraFrame, gray, COLOR_BGR2GRAY);
		subtract(gray, blue, yellow);
		subtract(blue, gray, blue);
		subtract(red, gray, red);
		subtract(yellow, red, yellow);
		subtract(red, yellow, red);
		
		//multiply(red, red, red);
		//multiply(blue, blue, blue);
		//multiply(yellow, yellow, yellow);
		if (imageShownSlider == 1) {
    		imshow("Original Image", cameraFrame);
			imshow("Gray", gray);
			imshow("Ball", red);
			imshow("Blue Goal", blue);
			imshow("Yellow Goal", yellow);
		}
		//imshow("Field", green);
		//imshow("Walls", black);
		
		//frameCounter++;
		//std::time_t timeNow = std::time(0) - timeBegin;
		//std::time_t timeBegin = std::time(0);
		//if (timeNow - tick >= 1) {
		//            tick++;
		//			cout << "Frames per second: " << frameCounter << endl;
		//			frameCounter = 0;
		//}
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