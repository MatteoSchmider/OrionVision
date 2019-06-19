/*#include <stdio.h>

int main (int argc, char **argv) 
{
	printf("Hello, world!\n");
	return 0;
}*/

#include <iostream>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;
int main(int argc, char **argv) {
    //Capture stream from webcam.
    VideoCapture capture(0);

    //Check if we can get the webcam stream.
    if(!capture.isOpened())
    {
        printf("Could not open camera\n");
        return -1;
    }
    printf("Opened Camera\n");
    while (true)
    {
		Mat cameraFrame;
		capture.read(cameraFrame);
		imshow("Cam View", cameraFrame);
    	if (waitKey(50) >= 0)
			break;
	}
 	return 0;
}
