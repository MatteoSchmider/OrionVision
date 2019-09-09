#define PI 3.141592
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

#define CENTER_X 332
#define CENTER_Y 252

#define DRIVING_FRONT_X 416

Mat gray(480, 640, CV_8UC1);
Mat blue(480, 640, CV_8UC1);
Mat blueNormalized(480, 640, CV_8UC1);
Mat blueEroded(480, 640, CV_8UC1);
Mat blueThreshold(480, 640, CV_8UC1);
Mat green(480, 640, CV_8UC1);
Mat greenNormalized(480, 640, CV_8UC1);
Mat greenThreshold(480, 640, CV_8UC1);
Mat red(480, 640, CV_8UC1);
Mat redNormalized(480, 640, CV_8UC1);
Mat redThreshold(480, 640, CV_8UC1);
Mat yellow;//(480, 640, CV_8UC1);
Mat yellowNormalized;//480, 640, CV_8UC1);
Mat yellowEroded(480, 640, CV_8UC1);
Mat yellowThreshold(480, 640, CV_8UC1);

Mat mask(480, 640, CV_8UC1);
Mat cameraFrame(480, 640, CV_8UC3);
Mat cameraFrameNoMask(480, 640, CV_8UC3);
Mat cameraFrameNoGamma(480, 640, CV_8UC3);
Mat cameraFrameNoBlur(480, 640, CV_8UC3);
Mat oneMat = Mat::ones(480, 640, CV_8UC1);

Mat channels[3];
Mat seg_channels[3];

int gammaSlider = 130;
int imageShownSlider = 0;
int threshold_red_slider = 25;
int threshold_blue_slider = 3;
int threshold_yellow_slider = 20;

double minr = 0, maxr = 0, minb = 0, maxb = 0, miny = 0, maxy = 0;
long mainCounter = 0, processCounter = 0, camCounter = 0;


int16_t ballXcam = 0, ballYcam = 0;
double ballX = 0, ballY = 0, ballRadius = 0, ballAngle = 0;
bool ballVisible = false;

int16_t goalBX = 0, goalBY = 0;
bool goalBVisible = false;

int16_t goalYX = 0, goalYY = 0;
bool goalYVisible = false;

int fd;

char teensyByte = 0;
bool robotOnField = false;

void wb(Mat& in) {
        double min, max;
        minMaxLoc(in, &min, &max, NULL, NULL);
        double scale = (int)(1.0 / (max - min));
        addWeighted(in, 1.0, oneMat, -min, scale, in);
}
const double EPS = 1E-9;

double sqr(double a) {
        return a * a;
}
double tangents2() {
        double r = -3.0 - 10.5;
        double z = sqr(ballX) + sqr(ballY);
        double d = z - sqr(r);
        if (d < -EPS) return 1000.0;
        d = sqrt(abs(d));
        double a = ((ballX * r) + (ballY * d)) / z;
        double b = ((ballY * r) - (ballX * d)) / z;
        return atan2(-a, b) * 180 / PI;
}
double tangents3() {
        double r = 3.0 + 10.5;
        double z = sqr(ballX) + sqr(ballY);
        double d = z - sqr(r);
        if (d < -EPS) return 1000.0;
        d = sqrt(abs(d));
        double a = ((ballX * r) + (ballY * d)) / z;
        double b = ((ballY * r) - (ballX * d)) / z;
        return atan2(-a, b) * 180 / PI;
}

void prepareFrame() {
        blur(cameraFrameNoMask, cameraFrameNoMask, Size(3,3));
        cameraFrameNoMask.copyTo(cameraFrameNoBlur, mask);

        Mat non_sat;
        cvtColor(cameraFrameNoBlur, non_sat, COLOR_BGR2HSV);
        split(non_sat, channels);
        multiply(channels[1], Scalar(gammaSlider / 100.0), channels[1]);
        merge(channels, 3, non_sat);
        cvtColor(non_sat, cameraFrameNoBlur, COLOR_HSV2BGR);

        split(cameraFrameNoBlur, channels);
        wb(channels[0]);
        wb(channels[1]);
        wb(channels[2]);
        blue = channels[0];
        green = channels[1];
        red = channels[2];
        merge(channels, 3, cameraFrame);
}

void normalizeChannels() {
        cvtColor(cameraFrame, gray, COLOR_BGR2GRAY);
        Mat temp;
        Mat hsv;
        //blue goal
        //subtract(blue, red, blueNormalized);
        cvtColor(cameraFrame, hsv, COLOR_BGR2HSV);
        inRange(hsv, Scalar(70, 0, 0), Scalar(255, 255, 30), hsv);
        hsv = Mat(Scalar(255)) - hsv;
        subtract(blue, green, blueNormalized);
        subtract(blue, red, temp);
        multiply(blueNormalized, temp, blueNormalized);
        subtract(blueNormalized, hsv, blueNormalized);
        erode(blueNormalized, blueNormalized, Mat());
        dilate(blueNormalized, blueNormalized, Mat());
        //field
        subtract(green, gray, greenNormalized);
        //ball
        subtract(red, green, redNormalized);
        //yellow goal
        /*absdiff(red, green, yellow);
           subtract(red, yellow, yellow);
           subtract(yellow, blue, yellow);
           subtract(yellow, redNormalized, yellow);
           subtract(yellow * 10, blue, yellowNormalized);*/
        subtract(red, blue, yellow);
        subtract(green, blue, temp);
        multiply(yellow, temp, yellow);
        threshold(yellow, yellow, 254, 255, THRESH_BINARY);
        erode(yellow, yellow, Mat());
        erode(yellow, yellow, Mat());
        erode(yellow, yellow, Mat());
        dilate(yellow, yellow, Mat());
        dilate(yellow, yellow, Mat());
        dilate(yellow, yellow, Mat());
        addWeighted(yellow, 0.5, redNormalized, -5, 0, yellowNormalized);
}

void printTeensy() {
        char ballAngleLow = ((int)(ballAngle)) & 0xFF;
        char ballAngleHigh = (((int)(ballAngle)) >> 8) & 0xFF;
        char ballRadiusLow = ((int)(ballRadius)) & 0xFF;
        char ballRadiusHigh = (((int)(ballRadius)) >> 8) & 0xFF;
        char ballVis = ballVisible;

        char goalBXLow = ((int)(goalBX)) & 0xFF;
        char goalBXHigh = (((int)(goalBX)) >> 8) & 0xFF;
        char goalBYLow = ((int)(goalBY)) & 0xFF;
        char goalBYHigh = (((int)(goalBY)) >> 8) & 0xFF;
        char goalBVis = goalBVisible;

        char goalYXLow = ((int)(goalYX)) & 0xFF;
        char goalYXHigh = (((int)(goalYX)) >> 8) & 0xFF;
        char goalYYLow = ((int)(goalYY)) & 0xFF;
        char goalYYHigh = (((int)(goalYY)) >> 8) & 0xFF;
        char goalYVis = goalYVisible;

        cout << ballAngleLow << endl;
        serialPutchar(fd, ballAngleLow);
        serialPutchar(fd, ballAngleHigh);
        serialPutchar(fd, ballRadiusLow);
        serialPutchar(fd, ballRadiusHigh);
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

double pixelsToCm(double pixels) {
        double centimeters = 14.11251 - (0.0158725955 * (1 - exp(0.04170403 * pixels)));
        return centimeters;
}
double getAngle(double xInImage, double yInImage) {
        double x = xInImage - CENTER_X;
        double y = yInImage - CENTER_Y;
        return atan2(y, x) * 180 / PI;
}
double getRadius(double xInImage, double yInImage) {
        double x = xInImage - CENTER_X;
        double y = yInImage - CENTER_Y;
        double radiusDouble = sqrt((sqr(x)) + (sqr(y)));
        return pixelsToCm(radiusDouble);
}
void doContours() {
        double angleRadius[2];
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
                angleRadius[0] = getAngle(minRectBall[0].center.x, minRectBall[0].center.y);
                angleRadius[1] = getRadius(minRectBall[0].center.x, minRectBall[0].center.y);
                ballVisible = true;
                ballAngle = angleRadius[0];
                ballRadius = angleRadius[1];
                ballX = ballRadius * cos(ballAngle / (180 / PI));
                ballY = ballRadius * sin(ballAngle / (180 / PI));
                if (ballX < 10.5) {
                        if (ballY < 0) {
                                ballAngle = tangents2() - 180.0;
                        }
                        else {
                                ballAngle = tangents3() - 180.0;
                        }
                        ballX = ballRadius * cos(ballAngle / (180 / PI));
                        ballY = ballRadius * sin(ballAngle / (180 / PI));
                        line(cameraFrame, Point(CENTER_X, CENTER_Y), Point(CENTER_X + (ballX * 2.0), CENTER_Y + (ballY * 2.0)), Scalar(0, 255, 0));
                }
                else {
                        ballX -= 10.5;
                        ballAngle = atan2(ballY, ballX) * 180 / PI;
                        ballRadius = sqrt((ballX * ballX) + (ballY * ballY));
                        line(cameraFrame, Point(DRIVING_FRONT_X, CENTER_Y), Point(DRIVING_FRONT_X + ballX, CENTER_Y + ballY), Scalar(0, 255, 0));
                }
                cout << "Ball X: " << ballX << endl;
                cout << "Ball Y: " << ballY << endl;
                cout << "Ball Radius: " << ballRadius << endl;
                cout << "Ball Angle: " << ballAngle << endl;
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
                angleRadius[0] = getAngle(minRectBG[0].center.x, minRectBG[0].center.y);
                angleRadius[1] = getRadius(minRectBG[0].center.x, minRectBG[0].center.y);
                goalBVisible = true;
                goalBX = angleRadius[1] * cos(angleRadius[0] / (180 / PI));
                goalBY = angleRadius[1] * sin(angleRadius[0] / (180 / PI));

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
                angleRadius[0] = getAngle(minRectYG[0].center.x, minRectYG[0].center.y);
                angleRadius[1] = getRadius(minRectYG[0].center.x, minRectYG[0].center.y);
                goalYVisible = true;
                goalYX = angleRadius[1] * cos(angleRadius[0] / (180 / PI));
                goalYY = angleRadius[1] * sin(angleRadius[0] / (180 / PI));

        }
        else {goalBVisible = false;}
}

void processFrame() {
        prepareFrame();
        normalizeChannels();
        erode(blueNormalized, blueEroded, getStructuringElement(MORPH_RECT, Size(5, 5)));
        //dilate(yellowNormalized, yellowNormalized, Mat());
        //erode(yellowNormalized, yellowNormalized, Mat());
        //erode(yellowNormalized, yellowNormalized, Mat());
        //erode(yellowNormalized, yellowEroded, Mat());
        double redTresh = (double) threshold_red_slider;
        double blueTresh = (double) threshold_blue_slider;
        double yellowTresh = (double) threshold_yellow_slider;
        threshold(redNormalized, redThreshold, redTresh, 255, THRESH_BINARY);
        threshold(blueEroded, blueThreshold, blueTresh, 255, THRESH_BINARY);
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
        thread image_getter(getFrames);

        mask = imread("mask.png", IMREAD_COLOR);

        fd = serialOpen("/dev/ttyS1", 115200);

        // Create a window
        namedWindow("Original Image", 1);
        createTrackbar("Gamma", "Original Image", &gammaSlider, 500);
        createTrackbar("Preview Images", "Original Image", &imageShownSlider, 5);
        createTrackbar("Red Treshold multiplier", "Original Image", &threshold_red_slider, 255);
        createTrackbar("Blue Treshold multiplier", "Original Image", &threshold_blue_slider, 255);
        createTrackbar("Yellow Treshold multiplier", "Original Image", &threshold_yellow_slider, 255);
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
                        imshow("Original Image", redNormalized * 4);
                        break;
                }
                case 4: {
                        imshow("Original Image", blueEroded * 4);
                        break;
                }
                case 5: {
                        imshow("Original Image", yellowNormalized * 4);
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
}
