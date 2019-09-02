#include <iostream>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core.hpp"
#include <opencv2/xphoto/white_balance.hpp>
#include <unistd.h>
#include <thread>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <cmath>

using namespace cv;
using namespace std;

#define CENTER_X 310
#define CENTER_Y 255
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

int gammaSlider = 50;
int imageShownSlider = 0;
int threshold_red_slider = 52;
int threshold_blue_slider = 7;
int threshold_yellow_slider = 35;

double minr = 0, maxr = 0, minb = 0, maxb = 0, miny = 0, maxy = 0;
long mainCounter = 0, processCounter = 0, camCounter = 0;

int16_t ballX = 0, ballY = 0;
bool ballVisible = false;

int16_t goalBX = 0, goalBY = 0;
bool goalBVisible = false;

int16_t goalYX = 0, goalYY = 0;
bool goalYVisible = false;

int fd;

char teensyByte = 0;
bool robotOnField = false;

void SimplestCB(Mat& in, Mat& out, float percent) {
        Mat lookUpTable(1, 256, CV_8U);
        uchar* p = lookUpTable.ptr();
        for( int i = 0; i < 256; ++i)
            p[i] = saturate_cast<uchar>(pow(i / 255.0, gamma_) * 255.0);
        Mat out = in.clone();
        LUT(in, lookUpTable, out);
}

void prepareFrame() {
        /*Ptr<xphoto::GrayworldWB> var = xphoto::createGrayworldWB();
           var->setSaturationThreshold(gammaSlider);
           var->balanceWhite(cameraFrameNoMask, cameraFrameNoMask);*/
        SimplestCB(cameraFrameNoMask, cameraFrameNoMask, (float) gammaSlider);
        cameraFrameNoMask.copyTo(cameraFrameNoBlur);//, mask);
        //blur
        blur(cameraFrameNoBlur, cameraFrame, Size(1, 1));
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

void printTeensy() {
        char ballXLow = ballX & 0xFF;
        char ballXHigh = (ballX >> 8) & 0xFF;
        char ballYLow = ballX & 0xFF;
        char ballYHigh = (ballX >> 8) & 0xFF;
        char ballVis = ballVisible;

        char goalBXLow = goalBX & 0xFF;
        char goalBXHigh = (goalBX >> 8) & 0xFF;
        char goalBYLow = goalBY & 0xFF;
        char goalBYHigh = (goalBY >> 8) & 0xFF;
        char goalBVis = goalBVisible;

        char goalYXLow = goalYX & 0xFF;
        char goalYXHigh = (goalYX >> 8) & 0xFF;
        char goalYYLow = goalYY & 0xFF;
        char goalYYHigh = (goalYY >> 8) & 0xFF;
        char goalYVis = goalYVisible;

        serialPutchar(fd, ballXLow);
        serialPutchar(fd, ballXHigh);
        serialPutchar(fd, ballYLow);
        serialPutchar(fd, ballYHigh);
        serialPutchar(fd, ballVis);

        serialPutchar(fd, goalBXLow);
        serialPutchar(fd, goalBXHigh);
        serialPutchar(fd, goalBYLow);
        serialPutchar(fd, goalBYHigh);
        serialPutchar(fd, goalBVis);

        serialPutchar(fd, goalYXLow);
        serialPutchar(fd, goalYXHigh);
        serialPutchar(fd, goalYYLow);
        serialPutchar(fd, goalYYHigh);
        serialPutchar(fd, goalYVis);
}

void doContours() {
        vector<vector<Point> > contoursBall;
        vector<Vec4i> hierarchyBall;

        findContours(redThreshold, contoursBall, hierarchyBall, RETR_TREE, CHAIN_APPROX_SIMPLE);

        vector<RotatedRect> minRectBall(contoursBall.size());
        for(int i = 0; i < contoursBall.size(); i++) {
                minRectBall[i] = minAreaRect(Mat(contoursBall[i]));
        }
        for(size_t i = 0; i < contoursBall.size(); i++) {
                Scalar color = Scalar(0, 0, 255);
                //drawContours(cameraFrame, contoursBall, (int)i, color, 2, LINE_8, hierarchyBall, 0);
                Point2f rect_points[4]; minRectBall[i].points( rect_points );
                for( int j = 0; j < 4; j++ )
                        line(cameraFrame, rect_points[j], rect_points[(j+1)%4], color, 1, 8);
        }
        if (contoursBall.size() > 0) {
                ballX = (int) (minRectBall[0].center.x);
                ballX -= CENTER_X;
                ballY = (int) (minRectBall[0].center.y);
                ballY -= CENTER_Y;
                ballVisible = true;
                cout << "Ball X: " << ballX << endl;
                cout << "Ball Y: " << ballY << endl;
                int ballradius = (int) sqrt((ballX * ballX) + (ballY * ballY));
                //double angle = (double) tan(ballY / ballX) * 180;
                cout << "Ball Radius: " << ballradius << endl;
                //cout << "Ball Angle: " << angle << endl;

        }
        else {ballVisible = false;}

        vector<vector<Point> > contoursBG;
        vector<Vec4i> hierarchyBG;

        findContours(blueThreshold, contoursBG, hierarchyBG, RETR_TREE, CHAIN_APPROX_SIMPLE);

        vector<RotatedRect> minRectBG(contoursBG.size());
        for(int i = 0; i < contoursBG.size(); i++) {
                minRectBG[i] = minAreaRect(Mat(contoursBG[i]));
        }
        for(size_t i = 0; i < contoursBG.size(); i++) {
                Scalar color = Scalar(255, 0, 0);
                //drawContours(cameraFrame, contoursBG, (int)i, color, 2, LINE_8, hierarchyBG, 0);
                Point2f rect_points[4]; minRectBG[i].points( rect_points );
                for( int j = 0; j < 4; j++ )
                        line(cameraFrame, rect_points[j], rect_points[(j+1)%4], color, 1, 8);
        }
        if (contoursBG.size() > 0) {
                goalBX = (int) (minRectBG[0].center.x);
                goalBY = (int) (minRectBG[0].center.y);
                goalBVisible = true;

        }
        else {goalBVisible = false;}

        vector<vector<Point> > contoursYG;
        vector<Vec4i> hierarchyYG;

        findContours(yellowThreshold, contoursYG, hierarchyYG, RETR_TREE, CHAIN_APPROX_SIMPLE);

        vector<RotatedRect> minRectYG(contoursYG.size());
        for(int i = 0; i < contoursYG.size(); i++) {
                minRectYG[i] = minAreaRect(Mat(contoursYG[i]));
        }
        for(size_t i = 0; i < contoursYG.size(); i++) {
                Scalar color = Scalar(0, 255, 255);
                //drawContours(cameraFrame, contoursYG, (int)i, color, 2, LINE_8, hierarchyYG, 0);
                Point2f rect_points[4]; minRectYG[i].points( rect_points );
                for( int j = 0; j < 4; j++ )
                        line(cameraFrame, rect_points[j], rect_points[(j+1)%4], color, 1, 8);
        }
        if (contoursYG.size() > 0) {
                goalYX = (int) (minRectYG[0].center.x);
                goalYY = (int) (minRectYG[0].center.y);
                goalYVisible = true;

        }
        else {goalBVisible = false;}
}

void processFrame() {
        prepareFrame();
        normalizeChannels();
        double redTresh = maxr * (double)(threshold_red_slider / 100.0);
        double blueTresh = maxb * (double)(threshold_blue_slider / 100.0);
        double yellowTresh = maxy * (double)(threshold_yellow_slider / 100.0);
        threshold(redNormalized, redThreshold, redTresh, 255, THRESH_BINARY);
        threshold(blueNormalized, blueThreshold, blueTresh, 255, THRESH_BINARY);
        threshold(yellowNormalized, yellowThreshold, yellowTresh, 255, THRESH_BINARY);
        doContours();
}

void processFrames() {
        while(true) {
                processFrame();
                printTeensy();
                //cout << "Image Processing Thread: " << processCounter << endl;
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
                //cout << "Camera Thread: " << camCounter << endl;
                camCounter++;
        }
}

int main(int argc, const char * argv[]) {

        //Capture stream from webcam.
        thread image_getter(getFrames);

        mask = imread("mask.png", IMREAD_COLOR);

        fd = serialOpen("/dev/ttyS1", 115200);

        // Create a window
        namedWindow("Original Image", 1);
        createTrackbar("Gamma", "Original Image", &gammaSlider, 500);
        createTrackbar("Preview Images", "Original Image", &imageShownSlider, 5);
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

        while (true) {
                switch (imageShownSlider) {
                case 0: {

                        break;
                }
                case 1: {
                        imshow("Original Image", cameraFrame);
                        break;
                }
                case 2: {
                        seg_channels[0] = blueNormalized * 10;
                        seg_channels[1] = greenNormalized * 25;
                        seg_channels[2] = redNormalized * 2;
                        Mat seg_img;
                        merge(seg_channels, 3, seg_img);
                        imshow("Original Image", seg_img);
                        break;
                }
                case 3: {
                        imshow("Original Image", redThreshold);
                        break;
                }
                case 4: {
                        imshow("Original Image", blueThreshold);
                        break;
                }
                case 5: {
                        imshow("Original Image", yellowThreshold);
                        break;
                }
                case 6: {
                        imshow("Original Image", cameraFrame);
                        imshow("Ball", redThreshold);
                        imshow("Blue Goal", blueThreshold);
                        imshow("Yellow Goal", yellowThreshold);
                        break;
                }
                }
                //cout << "Main Thread: " << mainCounter << endl;
                mainCounter++;
                char key = (char) waitKey(20);
                if (key == 'q' || key == 27)
                {
                        break;
                }
                if (serialDataAvail(fd)) {
                        teensyByte = serialGetchar(fd);
                        fflush(stdout);
                }
                robotOnField = (teensyByte == 1);
        }
        // return 0;
        // int fd;
        // if ((fd = serialOpen ("/dev/ttyS1", 115200)) < 0) {
        //         fprintf(stderr, "Unable to open serial device: %s\n", strerror (errno));
        //         return 1;
        // }
        // // Loop, getting and printing characters
        // for (;;) {
        //         if (serialDataAvail(fd)) {
        //                 putchar(serialGetchar(fd));
        //                 fflush(stdout);
        //         }
        // }
        // serialPutchar(fd, count);
        // return 0;
}
