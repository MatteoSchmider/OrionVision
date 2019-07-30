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

Mat gray;
Mat blue, blueTreshold;
Mat green, greenTreshold;
Mat red, redTreshold;
Mat yellow, yellowTreshold;

Mat mask;
Mat cameraFrame;
Mat cameraFrameNoMask;

Mat channels[3];

double fps1 = 0;
double fps2 = 0;
double fps3 = 0;
double totalFps = 0;
long totalFpsCount = 1;

auto start;
auto end;

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
        cameraFrameNoMask.copyTo(cameraFrame, mask);
        //gamma
        double gamma = gammaSlider / 100.0;
        cout << "Gamma: " << gamma << endl;
        cameraFrame = correctGamma(cameraFrame, gamma);
        //blur
        blur(cameraFrame, cameraFrame, Size(5, 5));
}

void normalizeChannels() {
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
}

void processFrame() {
        prepareFrame();
        normalizeChannels();
        threshold(red, redTreshold, maxr * threshold_red_slider / 100, 255, THRESH_BINARY);
        threshold(blue, blueTreshold, maxb * threshold_blue_slider / 100, 255, THRESH_BINARY);
        threshold(yellow, yellowTreshold, maxy * threshold_yellow_slider / 100, 255, THRESH_BINARY);
}

void processFrames() {
        while(true) {
                processFrame();
        }
}

int main(int argc, const char * argv[]) {

        //Capture stream from webcam.
        VideoCapture capture("rkcamsrc io-mode=4 isp-mode=2A ! video/x-raw,format=NV12,width=640,height=480 ! videoconvert ! appsink");
        mask = imread("mask.png", IMREAD_COLOR);

        //Check if we can get the webcam stream.
        if(!capture.isOpened()) {
                cout << "Could not open camera" << endl;
                return -1;
        }

        // Create a window
        namedWindow("Original Image", 1);
        int gammaSlider = 100;
        createTrackbar("Gamma", "Original Image", &gammaSlider, 500);
        int imageShownSlider = 1;
        createTrackbar("Preview Images", "Original Image", &imageShownSlider, 1);
        int threshold_red_slider = 80;
        createTrackbar("Red Treshold multiplier", "Original Image", &threshold_red_slider, 100);
        int threshold_blue_slider = 80;
        createTrackbar("Blue Treshold multiplier", "Original Image", &threshold_blue_slider, 100);
        int threshold_yellow_slider = 80;
        createTrackbar("Yellow Treshold multiplier", "Original Image", &threshold_yellow_slider, 100);

        for (int i = 0; i < 6; i++) {
                capture >> cameraFramenomask;
        }

        prepareFrame();

        normalizeChannels();

        double minr, maxr, minb, maxb, miny, maxy;
        minMaxLoc(red, &minr, &maxr, NULL, NULL);
        minMaxLoc(blue, &minb, &maxb, NULL, NULL);
        minMaxLoc(yellow, &miny, &maxy, NULL, NULL);

        thread image_processor(&processFrames, this);
        while (true) {
                start = chrono::steady_clock::now();

                //get frame
                capture >> cameraFrameNoMask;

                if (imageShownSlider == 1) {
                        imshow("Original Image", cameraFrame);
                        channels[0] = blue * 10;
                        channels[1] = green * 25;
                        channels[2] = red * 2;
                        Mat seg_img;
                        merge(channels, 3, seg_img);
                        imshow("Segmented Image", seg_img);
                        imshow("Ball", redThreshold);
                        imshow("Blue Goal", blueThreshold);
                        imshow("Yellow Goal", yellowThreshold);
                }

                totalFps += fps3;
                fps1 = fps2;
                fps2 = fps3;
                end = chrono::steady_clock::now();
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
