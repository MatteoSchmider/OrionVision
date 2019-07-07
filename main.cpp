#include <iostream>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core.hpp"

using namespace cv;
using namespace std;

Mat gray;
Mat blue;
Mat green;
Mat red;
Mat yellow;
Mat black;
Mat ryG;
Mat gyG;

Mat cameraFrame;

Mat channels[3];

int main(int argc, const char * argv[]) {
    //Capture stream from webcam.
    VideoCapture capture("rkcamsrc io-mode=4 isp-mode=2A ! video/x-raw,format=NV12,width=640,height=480 ! videoconvert ! appsink");

    //Check if we can get the webcam stream.
    if(!capture.isOpened()) {
        cout << "Could not open camera" << endl;
        return -1;
    }
	long frameCounter = 0;
	std::time_t timeBegin = std::time(0);
	int tick = 0;
    while (true) {
        //Read an image from the camera.
        capture.read(cameraFrame);
		
		//cameraFrame = imread("/Users/matteoschmider/Desktop/Foto.png", IMREAD_COLOR);
		
		split(cameraFrame, channels);
		blue = channels[0];
		green = channels[1];
		red = channels[2];
		
		cvtColor(cameraFrame, gray, COLOR_BGR2GRAY);
		//subtract(blue, gray, blue);
		//subtract(green, gray, green);
		//subtract(red, gray, red);
		
		divide(red, gray, red);
		divide(green, gray, green);
		divide(blue, gray, blue);
		
		absdiff(channels[2], channels[1], yellow);
		//equalizeHist(src, dst);
		
		//multiply(red, red, red);
		//threshold(red, red, 169, 255, THRESH_BINARY);
		
		/*extractChannel(cameraFrame, red, 2);
		extractChannel(cameraFrame, green, 1);
		extractChannel(cameraFrame, blue, 0);*/
		
		//addWeighted(blue, -1.0, blue, 0.0, 255.0, yellow);
		
		
		//subtract(yellow, channels[0], yellow);
		// above equals: yellow = 1 - blue
		
		absdiff(Scalar(255), gray, black);
		
    	imshow("Original Image", cameraFrame);
		imshow("Ball", red);
		imshow("Blue Goal", blue);
		imshow("Yellow Goal", yellow);
		//imshow("Field", green);
		//imshow("Walls", black);
		
		frameCounter++;
		std::time_t timeNow = std::time(0) - timeBegin;
		if (timeNow - tick >= 1) {
		            tick++;
		            cout << "Frames per second: " << frameCounter << endl;
					frameCounter = 0;
		}
		char key = (char) waitKey(1);
        if (key == 'q' || key == 27)
        {
            break;
        }
	}
	return 0;
}