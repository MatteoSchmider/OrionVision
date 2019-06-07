/*#include <stdio.h>

int main (int argc, char **argv) 
{
	printf("Hello, world!\n");
	return 0;
}*/

#include <iostream>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core.hpp"
using namespace cv;
using namespace std;
int main(int argc, char **argv) {
    //Capture stream from webcam.
    VideoCapture capture(-1);

    //Check if we can get the webcam stream.
    if(!capture.isOpened())
    {
        cout << "Could not open camera" << endl;
        return -1;
    }
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
//apt-get install uv4l uv4l-demos uv4l-dummy uv4l-mjpegstream uv4l-raspicam uv4l-raspicam-extras uv4l-raspidisp uv4l-server uv4l-uvc uv4l-xmpp-bridge uv4l-xscreen