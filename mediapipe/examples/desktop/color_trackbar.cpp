////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <iostream>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/formats/image_frame.h"
#include "mediapipe/framework/formats/image_frame_opencv.h"
#include "mediapipe/framework/port/file_helpers.h"
#include "mediapipe/framework/port/opencv_highgui_inc.h"
#include "mediapipe/framework/port/opencv_imgproc_inc.h"
#include "mediapipe/framework/port/opencv_video_inc.h"
#include "mediapipe/framework/port/parse_text_proto.h"
#include "mediapipe/framework/port/status.h"
#include "mediapipe/gpu/gl_calculator_helper.h"
#include "mediapipe/gpu/gpu_buffer.h"
#include "mediapipe/gpu/gpu_shared_data_internal.h"
#include "mediapipe/framework/formats/detection.pb.h"
#include "mediapipe/framework/formats/landmark.pb.h"
#include "mediapipe/framework/formats/rect.pb.h"
#include <opencv2/features2d.hpp>


using namespace cv;
using namespace std;

int main( int argc, char** argv )
{
     VideoCapture cap(0); //capture the video from web cam

     if ( !cap.isOpened() )  // if not success, exit program
     {
          cout << "Cannot open the web cam" << endl;
          return -1;
     }

     namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

     int iLowH = 29; //0
     int iHighH = 136; //179

     int iLowS = 64; //0
     int iHighS = 255; //255

     int iLowV = 163; //0
     int iHighV = 255; //255

     //Create trackbars in "Control" window
     cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
     cvCreateTrackbar("HighH", "Control", &iHighH, 179);

     cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
     cvCreateTrackbar("HighS", "Control", &iHighS, 255);

     cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
     cvCreateTrackbar("HighV", "Control", &iHighV, 255);
     //cv::SimpleBlobDetector detector;

     // Setup SimpleBlobDetector parameters.
     SimpleBlobDetector::Params params;
     
     // Change thresholds
     params.minThreshold = 10;
     params.maxThreshold = 200;
     
     // Filter by Area.
     params.filterByArea = false;
     params.minArea = 1000;
     
     // Filter by Circularity
     params.filterByCircularity = false;
     //params.minCircularity = 0.1;
     
     // Filter by Convexity
     params.filterByConvexity = true;
     params.minConvexity = 0.87;
     
     // Filter by Inertia
     params.filterByInertia = false;
     params.minInertiaRatio = 0.01;
     
     #if CV_MAJOR_VERSION < 3   // If you are using OpenCV 2
          // Set up detector with params
          SimpleBlobDetector detector(params);
          // You can use the detector this way
          // detector.detect( im, keypoints);
     #else
     
     // Set up detector with params
     
     // SimpleBlobDetector::create creates a smart pointer. 
     // So you need to use arrow ( ->) instead of dot ( . )
     // detector->detect( im, keypoints);
     
     #endif
     while (true)
     {
          Mat imgOriginal;

          bool bSuccess = cap.read(imgOriginal); // read a new frame from video

          if (!bSuccess) //if not success, break loop
          {
               cout << "Cannot read a frame from video stream" << endl;
               break;
          }

          Mat imgHSV;
          
          cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
          
          Mat imgThresholded;


          inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
          cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

          //morphological opening (remove small objects from the foreground)
          erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
          dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

          //morphological closing (fill small holes in the foreground)
          dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
          erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
          // Set up the detector with default parameters.
          
          bitwise_not(imgThresholded, imgThresholded);
          // Detect blobs.
          std::vector<KeyPoint> keypoints;
          detector->detect( imgThresholded, keypoints);
          // Draw detected blobs as red circles.
          // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
          Mat im_with_keypoints;

          drawKeypoints( imgOriginal, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
          
          // Show blobs
          imshow("keypoints", im_with_keypoints );

          imshow("Thresholded Image", imgThresholded); //show the thresholded image
          //imshow("Original", imgOriginal); //show the original image

          if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
          {
               cout << "esc key is pressed by user" << endl;
               break; 
          }
     }

     return 0;

}