#include <iostream>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core.hpp"
#include <chrono>
#include <unistd.h>
#include <thread>

using namespace cv;
using namespace std;

Mat gray(480, 640, CV_8UC1);
Mat blue(480, 640, CV_8UC1);
Mat blueNormalized(480, 640, CV_8UC1);
Mat blueThreshold(480, 640, CV_8UC1);
Mat green(480, 640, CV_8UC1);
Mat greenNormalized(480, 640, CV_8UC1);
Mat greenThreshold(480, 640, CV_8UC1);
Mat red(480, 640, CV_8UC1);
Mat redNormalized(480, 640, CV_8UC1);
Mat redThreshold(480, 640, CV_8UC1);
Mat yellow(480, 640, CV_8UC1);
Mat yellowNormalized(480, 640, CV_8UC1);
Mat yellowThreshold(480, 640, CV_8UC1);

Mat mask(480, 640, CV_8UC1);
Mat cameraFrame(480, 640, CV_8UC3);
Mat cameraFrameNoMask(480, 640, CV_8UC3);
Mat cameraFrameNoGamma(480, 640, CV_8UC3);
Mat cameraFrameNoBlur(480, 640, CV_8UC3);

Mat channels[3];
Mat seg_channels[3];

double fps1 = 0;
double fps2 = 0;
double fps3 = 0;
double totalFps = 0;
long totalFpsCount = 1;

auto startTime = chrono::steady_clock::now();
auto endTime = chrono::steady_clock::now();

int gammaSlider = 100;
int imageShownSlider = 1;
int threshold_red_slider = 80;
int threshold_blue_slider = 80;
int threshold_yellow_slider = 80;

double minr = 0, maxr = 0, minb = 0, maxb = 0, miny = 0, maxy = 0;
long mainCounter = 0, processCounter = 0, camCounter = 0;

Mat correctGamma(Mat& img, double gamma) {
        double inverse_gamma = 1.0 / gamma;
        Mat lut_matrix(1, 256, CV_8UC1 );
        uchar * ptr = lut_matrix.ptr();
        for( int i = 0; i < 256; i++ )
                ptr[i] = (int)( pow( (double) i / 255.0, inverse_gamma ) * 255.0 );
        Mat result;
        LUT( img, lut_matrix, result );
        return result;
}

void prepareFrame() {
        cameraFrameNoMask.copyTo(cameraFrameNoGamma, mask);
        //Gamma
        double Gamma = gammaSlider / 100.0;
        cout << "Gamma: " << Gamma << endl;
        cameraFrameNoBlur = correctGamma(cameraFrameNoGamma, Gamma);
        //blur
        blur(cameraFrameNoBlur, cameraFrame, Size(5, 5));
}

void normalizeChannels() {
        split(cameraFrame, channels);
        blue = channels[0];
        green = channels[1];
        red = channels[2];

        cvtColor(cameraFrame, gray, COLOR_BGR2GRAY);
        subtract(gray, blue, yellow);
        subtract(blue, gray, blueNormalized);
        subtract(red, gray, red);
        subtract(yellow, redNormalized, yellowNormalized);
        subtract(red, yellowNormalized, redNormalized);
        subtract(green, gray, greenNormalized);
}

void processFrame() {
        prepareFrame();
        normalizeChannels();
        threshold(redNormalized, redThreshold, maxr * threshold_red_slider / 100, 255, THRESH_BINARY);
        threshold(blueNormalized, blueThreshold, maxb * threshold_blue_slider / 100, 255, THRESH_BINARY);
        threshold(yellowNormalized, yellowThreshold, maxy * threshold_yellow_slider / 100, 255, THRESH_BINARY);
}

void processFrames() {
        while(true) {
                processFrame();
                cout << "Image Processing Thread: " << processCounter << endl;
                processCounter++;
        }
}

void getFrames() {
        VideoCapture capture("rkcamsrc io-mode=4 isp-mode=2A ! video/x-raw,format=NV12,width=640,height=480 ! videoconvert ! appsink");
        if(!capture.isOpened()) {
                cout << "Could not open camera" << endl;
        }
        while(true) {
                capture >> cameraFrameNoMask;
                cout << "Camera Thread: " << camCounter << endl;
                camCounter++;
        }
}

int main(int argc, const char * argv[]) {

        //Capture stream from webcam.
        thread image_getter(getFrames);

        mask = imread("mask.png", IMREAD_COLOR);

        //Check if we can get the webcam stream.


        // Create a window
        namedWindow("Original Image", 1);
        createTrackbar("Gamma", "Original Image", &gammaSlider, 500);
        createTrackbar("Preview Images", "Original Image", &imageShownSlider, 1);
        createTrackbar("Red Treshold multiplier", "Original Image", &threshold_red_slider, 100);
        createTrackbar("Blue Treshold multiplier", "Original Image", &threshold_blue_slider, 100);
        createTrackbar("Yellow Treshold multiplier", "Original Image", &threshold_yellow_slider, 100);

        prepareFrame();

        normalizeChannels();

        minMaxLoc(red, &minr, &maxr, NULL, NULL);
        minMaxLoc(blue, &minb, &maxb, NULL, NULL);
        minMaxLoc(yellow, &miny, &maxy, NULL, NULL);
        cout << "Just before threading!" << endl;
        thread image_processor(processFrames);
        //image_processor.join();
        while (true) {
                startTime = chrono::steady_clock::now();
                cout << "Main Thread!" << endl;
                //get frame

                if (imageShownSlider == 1) {
                        imshow("Original Image", cameraFrame);
                        seg_channels[0] = blueNormalized * 10;
                        seg_channels[1] = greenNormalized * 25;
                        seg_channels[2] = redNormalized * 2;
                        Mat seg_img;
                        merge(seg_channels, 3, seg_img);
                        imshow("Segmented Image", seg_img);
                        imshow("Ball", redThreshold);
                        imshow("Blue Goal", blueThreshold);
                        imshow("Yellow Goal", yellowThreshold);
                }

                // totalFps += fps3;
                // fps1 = fps2;
                // fps2 = fps3;
                // endTime = chrono::steady_clock::now();
                // fps3 = chrono::duration_cast<chrono::milliseconds>(endTime - startTime).count();
                // fps3 = 1000 / fps3;
                // double avg3 = (fps1 + fps2 + fps3) / 3;
                // double totalavg = totalFps / totalFpsCount;
                // cout << "Current FPS: " << fps3 << endl;
                // cout << "Running Average (3) FPS: " << avg3 << endl;
                // cout << "Total Average FPS: " << totalavg << endl;
                // totalFpsCount++;
                cout << "Main Thread: " << mainCounter << endl;
                mainCounter++;
                char key = (char) waitKey(20);
                if (key == 'q' || key == 27)
                {
                        break;
                }
        }
        return 0;
}
