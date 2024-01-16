///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2022, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

/*********************************************************************
 ** This sample demonstrates how to capture a live 3D point cloud   **
 ** with the ZED SDK and display the result in an OpenGL window.    **
 *********************************************************************/

// ZED includes
#include <sl/Camera.hpp>
#include <math.h>       /* isnan, sqrt */

// Sample includes
//#include "GLViewer.hpp"


//Shared memory
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <chrono>
#include <thread>
#include <memory>
#include <mutex>


//...and for c++ standard library functions:
#include <iostream>
#include <stdexcept>
#include <algorithm>

#ifdef _WIN32
extern "C" { uint32_t GetACP(); }
#endif
#include <cstdlib>

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

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include <memory>
#include <mutex>

#include <sstream>
#include <iostream>
#include <stdio.h>


# define NUM_LANDMARKS 21
# define THRESHOLD_CLOSED 24
# define THRESHOLD_ACTIVE_MILIMETERS 700
# define C1 100
# define C2 585
# define C3 75
# define C4 750
# define C5 (C4-(C3*C1)/C1)/(1-C3/C1)
# define C6 (C2-C5)/C1
# define min_count_out 10 
# define COUNT_NOT_HANDS 10 
# define FOCAL_LENGTH 4
//is the amount of width and height that is going to be cropped
# define SCALE 2.0
# define CROP_RATIO 0.7/3.0
# define handness 0 // 0 for default (center), 1 for left, 2 for right
constexpr char kInputStream[] = "input_video";
constexpr char kWindowName[] = "MediaPipe";
constexpr char kOutputLandmarks[] = "hand_landmarks";
constexpr char kSharedMemorySegmentName[] = "SharedMemory";

int num_hands_bycolor = 1;
int iLowH = 29; //0
int iHighH = 136; //179

int iLowS = 64; //0
int iHighS = 255; //255

int iLowV = 163; //0
int iHighV = 255; //255

int width = 672;

// Hand states enumeration
enum HandState {
    neutral,
    opening,
    closing,
};

ABSL_FLAG(std::string, calculator_graph_config_file, "",
          "Name of file containing text format CalculatorGraphConfig proto.");
ABSL_FLAG(std::string, input_video_path, "",
          "Full path of video to load. "
          "If not provided, attempt to use a webcam.");
ABSL_FLAG(std::string, output_video_path, "",
          "Full path of where to save result (.mp4 only). "
          "If not provided, show result in a window.");

clock_t current_ticks, delta_ticks;
clock_t fps = 0;
// Mapping between MAT_TYPE and CV_TYPE
int getOCVtype(sl::MAT_TYPE type) {
    int cv_type = -1;
    switch (type) {
        case sl::MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
        case sl::MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
        case sl::MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
        case sl::MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
        case sl::MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
        case sl::MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
        case sl::MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
        case sl::MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
        default: break;
    }
    return cv_type;
}
cv::Mat slMat2cvMat(sl::Mat& input) {
    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), getOCVtype(input.getDataType()), 
            input.getPtr<sl::uchar1>(sl::MEM::CPU), input.getStepBytes(sl::MEM::CPU));
}

// Function to calculate the distance between two points
float calculateDistance(const std::pair<float, float>& point1, const std::pair<float, float>& point2) {
    float dx = point2.first - point1.first;
    float dy = point2.second - point1.second;
    return std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
}
// Function to calculate the rotation angle between two points
float calculateRotationAngle(const std::pair<float, float>& point1, const std::pair<float, float>& point2) {
    float dx = point2.first - point1.first;
    float dy = point2.second - point1.second;
    return std::atan2(dy, dx);
}
double movingAverage(std::vector<double>& data, double value,int windowSize = 8) {
    data.push_back(value);
    if (data.size() < windowSize) {
        // Handle the case where there are not enough elements
        std::cerr << "Error: Not enough elements for "<< windowSize << " window size." << std::endl;
        return data.back();
    }
    double sum = 0.0;
    for (int i = 0; i < windowSize; ++i) {
        sum += data[i];
    }
    data.erase(data.begin());
    return sum / windowSize;
}

void detectHandState(std::vector<float>& prevDistances, const std::pair<float, float>& thumbPos, const std::pair<float, 
    float>& indexPos,HandState& hand_state, int windowSize = 5, float threshold = 3.0f) {
    // 0 neutral, 1 opening, 2 closing
    float distance = calculateDistance(thumbPos, indexPos);

    // Add the current distance to the vector of previous distances
    prevDistances.push_back(distance);

    // Check if we have enough previous distances to make a decision (e.g., consider last 3 distances)
    if (prevDistances.size() < windowSize) {
        // Handle the case where there are not enough elements
        std::cerr << "Error: Not enough elements to determine state: "<< windowSize << " window size." << std::endl;
        hand_state = neutral;
        return;
    }
    float sum = 0.0;
    for (int i = 0; i < windowSize; ++i) {
        sum += prevDistances[i];
    }
    // Calculate the average distance for the last <windowSize> samples
    float avgDistance = sum / windowSize;
    std::cout<<"distance: "<<distance<<" avg: "<<avgDistance<<" ";
    // Check if the current distance is close enough to the average distance to be considered neutral
    if (std::abs(distance - avgDistance) <= threshold) {
        hand_state = neutral;
    } else if (distance > avgDistance) {
        hand_state = opening;
    } else if (distance < avgDistance) {
        hand_state = closing;
    }

    // Limit the size of the vector to store only the last 3 distances
    if (prevDistances.size() > 3) {
        prevDistances.erase(prevDistances.begin());
    }
    
}

cv::Mat slMat2cvMatGPU(sl::Mat& input) {
    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), getOCVtype(input.getDataType()), 
            input.getPtr<sl::uchar1>(sl::MEM::GPU), input.getStepBytes(sl::MEM::GPU));
}
float median(std::vector<float> &v)
{
    size_t n = v.size() / 2;
    //std::cout<<n<<std::endl;
    if (n == 0)
        return 0;
    else{
        nth_element(v.begin(), v.begin()+n, v.end());
        return v[n];
    }
}


bool isHandActive(const std::vector<float>& landmarks, sl::Mat& point_cloud,int start_index,int num_hands,
                    int& curr_count, float& multiplier,double& distance){
  if (landmarks.size() != NUM_LANDMARKS * 2 * num_hands) {
    std::cerr << "Invalid number of landmarks. Expected " << NUM_LANDMARKS * 2 *num_hands
              << " but got " << landmarks.size() << std::endl;
    return false;
  }
    std::vector<float> distances;
    for(int i = start_index; i < start_index + NUM_LANDMARKS * 2; i+=2){
        float depth_value=0;
        point_cloud.getValue((width-1)-landmarks[i],landmarks[i+1],&depth_value,sl::MEM::GPU);
        if (!std::isnan(depth_value) && !std::isinf(depth_value)){
            if (depth_value > 50 && depth_value < 2000){
                distances.push_back(depth_value);
            }
        }
    }
    float depth_value=0;
    point_cloud.getValue(300,200,&depth_value,sl::MEM::GPU);
    std::cout<<" "<<depth_value<<std::endl;
    // distances.push_back(depth_value);
    
    float median_depth = median(distances);
    if (median_depth>100 & median_depth<585)
        multiplier = 1;
    else{
        
    }
    distance = median_depth;
    std::cout<<distances.size()<<" "<<distance<<" ";
    if(median_depth > 100 && median_depth < THRESHOLD_ACTIVE_MILIMETERS){
        if(min_count_out < curr_count){
            curr_count++;
        }
        else{
            curr_count = 0;
            return true;
        }
    }
    else{
        return false;
    }
}
float calculate_new_size(int object_size, int focal_length, float distance_to_object, float new_distance){
    float new_object_size = (object_size * focal_length * new_distance) / (distance_to_object * 1000);
    return new_object_size;
}
bool isHandClosed(const std::vector<float>& landmarks, int start_index,int num_hands,int& curr_count){
  if (landmarks.size() != NUM_LANDMARKS * 2 * num_hands) {
    std::cerr << "Invalid number of landmarks. Expected " << NUM_LANDMARKS * 2 *num_hands
              << " but got " << landmarks.size() << std::endl;
    return false;
  }

  // Calculate the distance between the thumb and pinky finger.
  float thumb_x = landmarks[start_index+8];
  float thumb_y = landmarks[start_index + 9];
  float pinky_x = landmarks[start_index + 40];
  float pinky_y = landmarks[start_index + 41];
  float distance = std::sqrt((thumb_x - pinky_x) * (thumb_x - pinky_x) +
                             (thumb_y - pinky_y) * (thumb_y - pinky_y));

    std::cout<<distance<<" ";
  // If the distance is less than a certain threshold, the hand is considered closed.
    if(distance < THRESHOLD_CLOSED){
        if(min_count_out < curr_count){
            curr_count++;
        }
        else{
            curr_count = 0;
            return true;
        }
    }
    else{
        return false;
    }

}


void parseArgs(int argc, char **argv, sl::InitParameters& param);
absl::Status RunMPPGraph(int argc, char** argv) {
    struct shm_remove
    {
        shm_remove() { boost::interprocess::shared_memory_object::remove(kSharedMemorySegmentName); }
        ~shm_remove(){ boost::interprocess::shared_memory_object::remove(kSharedMemorySegmentName); }
    } remover;
        // Create the shared memory segment
        boost::interprocess::managed_shared_memory segment(boost::interprocess::create_only, kSharedMemorySegmentName, 65536);

        // Create the vector in shared memory
        typedef boost::interprocess::allocator<int, boost::interprocess::managed_shared_memory::segment_manager> ShmemAllocator;
        typedef boost::interprocess::vector<int, ShmemAllocator> MyVector;
        MyVector *myvector = segment.construct<MyVector>("MyVector")(segment.get_segment_manager());

    // Create the mutex in shared memory
    typedef boost::interprocess::interprocess_mutex Mutex;
    Mutex *mutex = segment.construct<Mutex>("Mutex")();

    // Create the condition variable in shared memory
    typedef boost::interprocess::interprocess_condition CondVar;
    CondVar *condvar = segment.construct<CondVar>("CondVar")();

    // Setup SimpleBlobDetector parameters.
    cv::SimpleBlobDetector::Params params;
        
    // Change thresholds
    params.minThreshold = 10;
    params.maxThreshold = 200;
        
    // Filter by Area.
    params.filterByArea = true;
    params.minArea = 20;
        
    // Filter by Circularity
    params.filterByCircularity = false;
    //params.minCircularity = 0.1;
        
    // Filter by Convexity
    params.filterByConvexity = true;
    params.minConvexity = 0.87;
        
    // Filter by Inertia
    params.filterByInertia = false;
    params.minInertiaRatio = 0.01;

     int iLowH = 38; //0
     int iHighH = 93; //179

     int iLowS = 96; //0
     int iHighS = 255; //255

     int iLowV = 32; //0
     int iHighV = 255; //255
    std::string calculator_graph_config_contents;

    MP_RETURN_IF_ERROR(mediapipe::file::GetContents(
        absl::GetFlag(FLAGS_calculator_graph_config_file),
        &calculator_graph_config_contents));
    LOG(INFO) << "Get calculator graph config contents: "
                << calculator_graph_config_contents;
    mediapipe::CalculatorGraphConfig config =
        mediapipe::ParseTextProtoOrDie<mediapipe::CalculatorGraphConfig>(
            calculator_graph_config_contents);

    LOG(INFO) << "Initialize the calculator graph.";
    mediapipe::CalculatorGraph graph;
    MP_RETURN_IF_ERROR(graph.Initialize(config));

    LOG(INFO) << "Initialize the GPU.";
    ASSIGN_OR_RETURN(auto gpu_resources, mediapipe::GpuResources::Create());
    MP_RETURN_IF_ERROR(graph.SetGpuResources(std::move(gpu_resources)));
    mediapipe::GlCalculatorHelper gpu_helper;
    gpu_helper.InitializeForTest(graph.GetGpuResources().get());

    LOG(INFO) << "Initialize the camera or load the video.";
    std::string camera_type = std::string(argv[2]);
    std::cout<<camera_type<<std::endl;
    std::string tracking_type = std::string(argv[3]);
    std::string fingers_info_to_send = std::string(argv[4]);


    //std::cout<<<<std::endl;
  if(camera_type.find("zed") != std::string::npos && tracking_type.find("mediapipe") != std::string::npos){
    std::cout<<"Let's use zed"<<std::endl;
    sl::Camera zed;
    // Set configuration parameters for the ZED
    sl::InitParameters init_parameters;
    init_parameters.depth_mode = sl::DEPTH_MODE::NEURAL;
    init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP; // OpenGL's coordinate system is right_handed
    init_parameters.sdk_verbose = 1;
    parseArgs(argc, argv, init_parameters);

    // Open the camera
    auto returned_state = zed.open(init_parameters);
    if (returned_state != sl::ERROR_CODE::SUCCESS) {
        //print("Camera Open", returned_state, "Exit program.");
        return absl::InternalError("Camera Open Error");
    }

    auto camera_config = zed.getCameraInformation().camera_configuration;
    auto stream = zed.getCUDAStream();
    // Allocation of 1 channel of float on GPU
    sl::Mat point_cloud(camera_config.resolution, sl::MAT_TYPE::F32_C1, sl::MEM::GPU);
    //   cv::VideoCapture capture;
    //   const bool load_video = !absl::GetFlag(FLAGS_input_video_path).empty();
    //   if (load_video) {
    //     capture.open(absl::GetFlag(FLAGS_input_video_path));
    //   } else {
    //     capture.open(0);
    //   }
    //   RET_CHECK(capture.isOpened());
    
    cv::VideoWriter writer;
    const bool save_video = !absl::GetFlag(FLAGS_output_video_path).empty();
    //   if (!save_video) {
    //     cv::namedWindow(kWindowName, /*flags=WINDOW_AUTOSIZE*/ 1);
    // #if (CV_MAJOR_VERSION >= 3) && (CV_MINOR_VERSION >= 2)
    //     capture.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    //     capture.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    //     capture.set(cv::CAP_PROP_FPS, 30);
    // #endif
    //   }

    LOG(INFO) << "Start running the calculator graph.";
    ASSIGN_OR_RETURN(mediapipe::OutputStreamPoller pPoller_landmarks,
                        graph.AddOutputStreamPoller(kOutputLandmarks));
    MP_RETURN_IF_ERROR(graph.StartRun({}));

    sl::RuntimeParameters runParameters;
    // Setting the depth confidence parameters
    //runParameters.confidence_threshold = 50;
    runParameters.confidence_threshold = 50;
    runParameters.sensing_mode = sl::SENSING_MODE::FILL;
    runParameters.texture_confidence_threshold = 100;

    LOG(INFO) << "Start grabbing and processing frames.";
    bool grab_frames = true;
    sl::Mat image_zed;
    int count_active_1 = 0;
    int count_active_2 = 0;
    int count_closed_1 = 0;
    int count_closed_2 = 0;
    double distance_1 = 0.0;
    double distance_2 = 0.0;
    int count_hand_not_found_1 = 0, count_hand_not_found_2 = 0;
    double t1, t2, t3, t4;
    double t5, t6, t7, t8;
    std::vector<double> hand1_l1, hand1_l2, hand1_l3, hand1_l4;
    std::vector<double> hand2_l1, hand2_l2, hand2_l3, hand2_l4;
    std::vector<float> hand1_finger_distances;
    
    width = camera_config.resolution.width;
    while (grab_frames) {
        // Main Loop
        // Check that a new image is successfully acquired
        if (zed.grab(runParameters) == sl::ERROR_CODE::SUCCESS) {
            // retrieve the current 3D coloread point cloud in GPU
            zed.retrieveImage(image_zed, sl::VIEW::LEFT); // Retrieve left image
            zed.retrieveMeasure(point_cloud, sl::MEASURE::DEPTH, sl::MEM::GPU);
            //viewer.updatePointCloud(point_cloud);
            current_ticks = clock();
            // Capture opencv camera or video frame.
            // cv::Mat camera_frame_raw;
            // capture >> camera_frame_raw;
            // if (camera_frame_raw.empty()) {
            // if (!load_video) {
            //     LOG(INFO) << "Ignore empty frames from camera.";
            //     continue;
            // }
            // LOG(INFO) << "Empty frame, end of video reached.";
            // break;
            // }
            cv::Mat camera_frame;

            cv::Mat image_cv = slMat2cvMat(image_zed);
            cv::cvtColor(image_cv, camera_frame, cv::COLOR_BGR2RGBA);
            //cv::flip(point_cloud, point_cloud, /*flipcode=HORIZONTAL*/ 1);
            //std::cout<<camera_frame.cols<<"  "<<camera_frame.rows<<std::endl;



            cv::flip(camera_frame, camera_frame, /*flipcode=HORIZONTAL*/ 1);
            

            {
                // Wrap Mat into an ImageFrame.
                auto input_frame = absl::make_unique<mediapipe::ImageFrame>(
                    mediapipe::ImageFormat::SRGBA, camera_frame.cols, camera_frame.rows,
                    mediapipe::ImageFrame::kGlDefaultAlignmentBoundary);
                cv::Mat input_frame_mat = mediapipe::formats::MatView(input_frame.get());
                camera_frame.copyTo(input_frame_mat);

                // Prepare and add graph input packet.
                size_t frame_timestamp_us = 
                (double)cv::getTickCount() / (double)cv::getTickFrequency() * 1e6;
                MP_RETURN_IF_ERROR(
                    gpu_helper.RunInGlContext([&input_frame, &frame_timestamp_us, &graph,
                                            &gpu_helper]() -> absl::Status {
                    // Convert ImageFrame to GpuBuffer.
                    auto texture = gpu_helper.CreateSourceTexture(*input_frame.get());
                    auto gpu_frame = texture.GetFrame<mediapipe::GpuBuffer>();
                    glFlush();
                    texture.Release();
                    // Send GPU image packet into the graph.
                    MP_RETURN_IF_ERROR(graph.AddPacketToInputStream(
                        kInputStream, mediapipe::Adopt(gpu_frame.release())
                                            .At(mediapipe::Timestamp(frame_timestamp_us))));
                    return absl::OkStatus();
                    }));

                // Get the graph result packet, or stop if that fails.
                // mediapipe::Packet packet;
                // if (!poller.Next(&packet)) break;
                // std::unique_ptr<mediapipe::ImageFrame> output_frame;
                // mediapipe::Packet multi_hand_landmarks_packet;
                //Landmark reading

                mediapipe::Packet packet_landmarks;
                std::vector<float> hand_landmarks;
                hand_landmarks.clear();
                //printf("poller is not null.\n");
                if(pPoller_landmarks.QueueSize() > 0){
                    if (pPoller_landmarks.Next(&packet_landmarks)){
                        std::vector<mediapipe::NormalizedLandmarkList> output_landmarks =
                        packet_landmarks.Get<std::vector<mediapipe::NormalizedLandmarkList>>();
                        for (int m = 0; m < output_landmarks.size(); ++m){
                            mediapipe::NormalizedLandmarkList single_hand_NormalizedLandmarkList = output_landmarks[m];
                            for (int i = 0; i < single_hand_NormalizedLandmarkList.landmark_size(); ++i){
                                const mediapipe::NormalizedLandmark landmark = single_hand_NormalizedLandmarkList.landmark(i);
                                
                                int x = landmark.x() * camera_frame.cols;
                                int y = landmark.y() * camera_frame.rows;
                                //int x = landmark.x();
                                //int y = landmark.y();
                                hand_landmarks.push_back(x);
                                hand_landmarks.push_back(y);
                            }
                        }
                        bool hand_1_closed = false;
                        bool hand_2_closed = false;
                        bool hand_1_active = false;
                        bool hand_2_active = false;
                        float multiplier = 1.0;
                        if (hand_landmarks.size()==42){
                            //printf("hand 1 is closed.\n");
                            hand_1_closed = isHandClosed(hand_landmarks,0,1,count_closed_1);
                            hand_1_active = isHandActive(hand_landmarks,point_cloud,0,1,count_active_1,multiplier,distance_1);
                            std::cout<<" "<<hand_1_closed<<"  "<<hand_1_active<<" dist: "<<distance_1<<std::endl;
                        }
                        else if(hand_landmarks.size()==84){
                            //printf("hand 2 is closed.\n");
                            hand_1_closed = isHandClosed(hand_landmarks,0,2,count_closed_1);
                            hand_1_active = isHandActive(hand_landmarks,point_cloud,0,2,count_active_1,  multiplier,distance_1);
                            hand_2_closed = isHandClosed(hand_landmarks, 42,2,count_closed_2);
                            hand_2_active = isHandActive(hand_landmarks,point_cloud,42,2,count_active_2,multiplier,distance_2);
                            std::cout<<hand_1_closed<<" "<<hand_1_active<<" "<<hand_2_closed<<"  "<<hand_2_active;
                        }
                        //printf("hand points size = %d.\n", hand_landmarks.size());
                        // Lock the mutex
                        std::unique_lock<Mutex> lock(*mutex);
                        // Write the data to shared memory
                        myvector->clear();
                        //for (int i = 0; i < 2; i++) {
                        if (hand_landmarks.size()==42){
                            if (count_hand_not_found_1 == 0 && count_hand_not_found_2 == 0){
                                hand1_l1.clear(); hand1_l2.clear(); hand1_l3.clear(); hand1_l4.clear();
                            }

                            if (distance_1<1000 && distance_1>0){
                                std::cout<<"pushing data 1 = "<<distance_1<<std::endl;
                                myvector->push_back(1); //num_hands
                                //HAND 1
                                myvector->push_back(int(hand_1_closed)); //hand 1 closed
                                myvector->push_back(int(hand_1_active)); //hand 1 closed
                                t1 = movingAverage(hand1_l1,hand_landmarks[16]* SCALE);
                                t2 = movingAverage(hand1_l2,hand_landmarks[17]* SCALE);
                                t3 = movingAverage(hand1_l3,hand_landmarks[8]* SCALE);
                                t4 = movingAverage(hand1_l4,hand_landmarks[9]* SCALE);
                                myvector->push_back(t1); //pointing finger
                                myvector->push_back(t2);
                                myvector->push_back(t3); //thumb finger
                                myvector->push_back(t4);
                                condvar->notify_one();
                                count_hand_not_found_1 = 0;
                                count_hand_not_found_2 ++;
                            }
                            else{
                                count_hand_not_found_1 ++;
                                count_hand_not_found_2 ++;
                            }
                        }
                        else if(hand_landmarks.size()==84){
                            if (distance_1<1000 && distance_1>0 && distance_2<1000 && distance_2>0){
                                myvector->push_back(2); //num_hands
                                //HAND 1
                                myvector->push_back(int(hand_2_closed)); //hand 2 closed
                                myvector->push_back(int(hand_2_active)); //hand 1 closed
                                t5 = movingAverage(hand1_l1,hand_landmarks[58]* SCALE); //pointing finger
                                t6 = movingAverage(hand1_l2,hand_landmarks[59]* SCALE);
                                t7 = movingAverage(hand1_l3,hand_landmarks[50]* SCALE); //thumb finger
                                t8 = movingAverage(hand1_l4,hand_landmarks[51]* SCALE);
                                myvector->push_back(t5); //pointing finger
                                myvector->push_back(t6);
                                myvector->push_back(t7); //thumb finger
                                myvector->push_back(t8);
                                //HAND 2
                                myvector->push_back(int(hand_1_closed)); //hand 1 closed
                                myvector->push_back(int(hand_1_active)); //hand 1 closed
                                t1 = movingAverage(hand2_l1,hand_landmarks[16]* SCALE);  //pointing finger  
                                t2 = movingAverage(hand2_l2,hand_landmarks[17]* SCALE);
                                t3 = movingAverage(hand2_l3,hand_landmarks[8]* SCALE); //thumb finger
                                t4 = movingAverage(hand2_l4,hand_landmarks[9]* SCALE);
                                myvector->push_back(t1); //pointing finger
                                myvector->push_back(t2);
                                myvector->push_back(t3); //thumb finger
                                myvector->push_back(t4);

                                condvar->notify_one();
                                count_hand_not_found_1 = 0;
                                count_hand_not_found_2 = 0;
                            }
                            // else if (distance_1<1000 && distance_1>0){
                            //     myvector->push_back(1); //num_hands
                            //     myvector->push_back(int(hand_1_closed)); //hand 1 closed
                            //     myvector->push_back(int(hand_1_active)); //hand 1 closed
                            //     t1 = movingAverage(hand1_l1,hand_landmarks[16]* SCALE);  //pointing finger  
                            //     t2 = movingAverage(hand1_l2,hand_landmarks[17]* SCALE);
                            //     t3 = movingAverage(hand1_l3,hand_landmarks[8]* SCALE); //thumb finger
                            //     t4 = movingAverage(hand1_l4,hand_landmarks[9]* SCALE);
                            //     myvector->push_back(t1); //pointing finger
                            //     myvector->push_back(t2);
                            //     myvector->push_back(t3); //thumb finger
                            //     myvector->push_back(t4);
                            //     condvar->notify_one();
                            //     count_hand_not_found_1 = 0;
                            //     count_hand_not_found_2 ++;

                            // }
                            // else if (distance_2<1000 && distance_2>0){
                            //     myvector->push_back(1); //num_hands
                            //     t5 = movingAverage(hand2_l1,hand_landmarks[58]* SCALE); //pointing finger
                            //     t6 = movingAverage(hand2_l2,hand_landmarks[59]* SCALE);
                            //     t7 = movingAverage(hand2_l3,hand_landmarks[50]* SCALE); //thumb finger
                            //     t8 = movingAverage(hand2_l4,hand_landmarks[51]* SCALE);
                            //     myvector->push_back(t5); //pointing finger
                            //     myvector->push_back(t6);
                            //     myvector->push_back(t7); //thumb finger
                            //     myvector->push_back(t8);
                            //     condvar->notify_one();
                            //     count_hand_not_found_2 = 0;
                            //     count_hand_not_found_1 ++;
                            // }
                        }
                        // Notify the condition variable
                    }
                }
                else{
                    count_hand_not_found_1 ++;
                    count_hand_not_found_2 ++;
                }
                // Clear the gesture array.
                if (count_hand_not_found_1>COUNT_NOT_HANDS){
                    hand1_l1.clear(); hand1_l2.clear(); hand1_l3.clear(); hand1_l4.clear();
                }
                if (count_hand_not_found_2>COUNT_NOT_HANDS){
                    hand2_l1.clear(); hand2_l2.clear(); hand2_l3.clear(); hand2_l4.clear();
                }
                //std::cout<<" "<<count_hand_not_found_1<<" "<<count_hand_not_found_2<<" "<<hand1_l1.size()<<" "<<hand2_l1.size()<<std::endl;
                // Press any key to exit.
                const int pressed_key = cv::waitKey(5);
                if (pressed_key >= 0 && pressed_key != 255) grab_frames = false;

                delta_ticks = clock() - current_ticks; //the time, in ms, that took to render the scene
                if(delta_ticks > 0)
                    fps = CLOCKS_PER_SEC / delta_ticks;
                std::cout << "FPS:"<<fps << std::endl;
            }
            //LOG(INFO) << fps;

        }
    }
        // free allocated memory before closing the ZED
        point_cloud.free();
        // close the ZED
        zed.close();
        LOG(INFO) << "Shutting down.";
        if (writer.isOpened()) writer.release();
        MP_RETURN_IF_ERROR(graph.CloseInputStream(kInputStream));
        return graph.WaitUntilDone();
  }
  else if(camera_type.find("zed") != std::string::npos && tracking_type.find("color") != std::string::npos){
    std::cout<<"Let's use zed with color"<<std::endl;
    sl::Camera zed;
    // Set configuration parameters for the ZED
    sl::InitParameters init_parameters;
    init_parameters.depth_mode = sl::DEPTH_MODE::NEURAL;
    init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP; // OpenGL's coordinate system is right_handed
    init_parameters.sdk_verbose = 1;
    parseArgs(argc, argv, init_parameters);

    // Open the camera
    auto returned_state = zed.open(init_parameters);
    if (returned_state != sl::ERROR_CODE::SUCCESS) {
        //print("Camera Open", returned_state, "Exit program.");
        return absl::InternalError("Camera Open Error");
    }

    auto camera_config = zed.getCameraInformation().camera_configuration;
    auto stream = zed.getCUDAStream();
    // Allocation of 1 channel of float on GPU
    // sl::Mat point_cloud(camera_config.resolution, sl::MAT_TYPE::F32_C1, sl::MEM::GPU);
    //   cv::VideoCapture capture;
    //   const bool load_video = !absl::GetFlag(FLAGS_input_video_path).empty();
    //   if (load_video) {
    //     capture.open(absl::GetFlag(FLAGS_input_video_path));
    //   } else {
    //     capture.open(0);
    //   }
    //   RET_CHECK(capture.isOpened());
    
    cv::VideoWriter writer;
    const bool save_video = !absl::GetFlag(FLAGS_output_video_path).empty();
    //  if (!save_video) {
    //     cv::namedWindow(kWindowName, /*flags=WINDOW_AUTOSIZE*/ 1);
    // #if (CV_MAJOR_VERSION >= 3) && (CV_MINOR_VERSION >= 2)
    //     capture.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    //     capture.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    //     capture.set(cv::CAP_PROP_FPS, 30);
    // #endif
    //   }
    std::cout<<"Let's use color based tracking"<<std::endl;
    cv::namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

    //Create trackbars in "Control" window
    cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("HighH", "Control", &iHighH, 179);

    cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "Control", &iHighS, 255);

    cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "Control", &iHighV, 255);

    sl::RuntimeParameters runParameters;
    // Setting the depth confidence parameters
    //runParameters.confidence_threshold = 50;
    runParameters.confidence_threshold = 50;
    runParameters.sensing_mode = sl::SENSING_MODE::FILL;
    runParameters.texture_confidence_threshold = 100;

    LOG(INFO) << "Start grabbing and processing frames.";
    bool grab_frames = true;
    sl::Mat image_zed;
    int count_active_1 = 0;
    int count_active_2 = 0;
    int count_closed_1 = 0;
    int count_closed_2 = 0;
    double distance_1 = 0.0;
    double distance_2 = 0.0;
    int count_hand_not_found_1 = 0, count_hand_not_found_2 = 0;
    double t1, t2, t3, t4;
    double rot1,c1_x,c1_y; //rotation and hand1 center
    double t5, t6, t7, t8;
    std::vector<double> hand1_l1, hand1_l2, hand1_l3, hand1_l4;
    std::vector<double> hand2_l1, hand2_l2, hand2_l3, hand2_l4;
    std::vector<double> hand1_rot, hand1_center_x,hand1_center_y;
    std::vector<float> hand1_finger_distances;

    width = camera_config.resolution.width;
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    std::vector<cv::KeyPoint> keypoints;
    bool hand_1_closed = false;
    bool hand_2_closed = false;
    bool hand_1_active = false;
    bool hand_2_active = false;
    int frame_count = 0;
    bool show_image = true;
    double scale_dueto_crop = 1.0/(CROP_RATIO*2.0);
    while (grab_frames) {
        // Main Loop
        // Check that a new image is successfully acquired
        if (zed.grab(runParameters) == sl::ERROR_CODE::SUCCESS) {
            // retrieve the current 3D coloread point cloud in GPU
            zed.retrieveImage(image_zed, sl::VIEW::LEFT); // Retrieve left image
            //zed.retrieveMeasure(point_cloud, sl::MEASURE::DEPTH, sl::MEM::GPU);
            //viewer.updatePointCloud(point_cloud);
            current_ticks = clock();
            // Press any key to exit.

            cv::Mat camera_frame;

            cv::Mat image_cv = slMat2cvMat(image_zed);

            cv::flip(image_cv, image_cv, /*flipcode=HORIZONTAL*/ 1);
            //std::cout<<CROP_RATIO/2.0<<std::endl;
            int roi_x = image_cv.cols / (1.0/(CROP_RATIO/2.0));  // 1/3 of the width
            int roi_y = image_cv.rows / (1.0/(CROP_RATIO/2.0));  // 1/3 of the height
            int roi_width = image_cv.cols * (CROP_RATIO*2.0);    // 2/3 of the width
            int roi_height = image_cv.rows * (CROP_RATIO*2.0);   // 2/3 of the height
            //std::cout<<roi_x<<"  "<<roi_y<<"  "<<roi_width<<"  "<<roi_height<<std::endl;

            // Create the rectangle ROI
            cv::Rect roi_rect(roi_x, roi_y, roi_width, roi_height);

            // Extract the ROI from the original image
            cv::Mat roi_img = image_cv(roi_rect);
            //cv::cvtColor(image_cv, camera_frame, cv::COLOR_BGR2RGBA);
            //cv::flip(point_cloud, point_cloud, /*flipcode=HORIZONTAL*/ 1);
            //std::cout<<camera_frame.cols<<"  "<<camera_frame.rows<<std::endl;

            //std::cout<<"Let's use color based tracking"<<std::endl;
            cv::Mat imgHSV;
            cv::Mat imgThresholded;

            cv::cvtColor(roi_img, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
            //if (!load_video) {
            // }
            // Setup a rectangle to define your region of interest
            // cv::Rect myROI(10, 10, 100, 100);

            // Crop the full image to that image contained by the rectangle myROI
            // cv::Mat croppedImage = image_HSV(myROI);

            cv::inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
            
            //morphological opening (remove small objects from the foreground)
            cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
            cv::dilate( imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 

            //morphological closing (fill small holes in the foreground)
            cv::dilate( imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 
            cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
            // Set up the detector with default parameters.
            cv::Mat hierarchy = cv::Mat();

            std::vector<std::vector<cv::Point> > contours;
            cv::Mat contourOutput = imgThresholded.clone();

            cv::findContours(contourOutput, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
            hierarchy.release();
            // filter them based on Area.  good to remove small dots
            double minArea = 30; 
            double maxArea = 900000;
            std::vector<std::vector<cv::Point> > GoodContours;
            // Note that this doesn't copy the data
            cv::bitwise_not(imgThresholded, imgThresholded);

            for (size_t idx = 0; idx < contours.size(); idx++) {
                double area = cv::contourArea(contours[idx]);
                // http://www.cplusplus.com/reference/limits/numeric_limits/
                if (area >= (minArea == -1 ? std::numeric_limits<double>::min() : minArea) && area <= (maxArea == -1 ? std::numeric_limits<double>::max() : maxArea)) {
                    GoodContours.push_back(contours.at(idx));
                }
            }

            //Draw the contours
            cv::Mat contourImage(roi_img.size(), CV_8UC3, cv::Scalar(0,0,0));
            cv::Scalar colors[3];
            colors[0] = cv::Scalar(255, 0, 0);
            colors[1] = cv::Scalar(0, 255, 0);
            colors[2] = cv::Scalar(0, 0, 255);


            for (size_t idx = 0; idx < GoodContours.size(); idx++) {
                cv::drawContours(contourImage, GoodContours, idx, colors[idx % 3]);
            }

            //cv::imshow("Input Image", currentImage);


            //std::cout << "num keypts: " << keypoints.size() << " countours size: "<<GoodContours.size();
            // Detect blobs.
            // if (true){
            //     detector->detect( imgThresholded, keypoints);
            //     frame_count =0;
            //     std::vector<cv::KeyPoint>::const_iterator it = keypoints.begin(), end = keypoints.end();
            //     for( ; it != end; ++it ) {
            //     std::cout << "  kpt x " << it->pt.x << ", y " << it->pt.y << ", size " << it->size << "\n";
            //     }
            // }
            {   
                //Find one finger
                if(fingers_info_to_send.find("centroid") != std::string::npos && GoodContours.size() == num_hands_bycolor*1){
                    std::vector<cv::Point> centers; 
                    for (int i=0; i<GoodContours.size(); i++){
                        cv::Moments M = cv::moments(GoodContours[i]);
                        cv::Point center(M.m10/M.m00, M.m01/M.m00);
                        centers.push_back(center);
                    }
                    hand_1_closed = false;
                    hand_1_active = true;
                    
                        myvector->push_back(1); //num_hands
                        //HAND 1
                        myvector->push_back(int(hand_1_closed)); //hand 1 closed
                        myvector->push_back(int(hand_1_active)); //hand 1 closed
                        // t1 = movingAverage(hand1_l1,keypoints[0].pt.x * SCALE);
                        // t2 = movingAverage(hand1_l2,keypoints[0].pt.y* SCALE);
                        // t3 = movingAverage(hand1_l3,keypoints[1].pt.x* SCALE,10);
                        // t4 = movingAverage(hand1_l4,keypoints[1].pt.y* SCALE,10);
                        t3 = movingAverage(hand1_l1,centers[0].x * scale_dueto_crop * SCALE,2);
                        t4 = movingAverage(hand1_l2,centers[0].y * scale_dueto_crop * SCALE,2);
                        t1 = movingAverage(hand1_l3,centers[0].x * scale_dueto_crop * SCALE,2);
                        t2 = movingAverage(hand1_l4,centers[0].y * scale_dueto_crop * SCALE,2);
                        
                        //Get hand state
                        std::pair index = std::make_pair(t1, t2);
                        std::pair thumb = std::make_pair(t3, t4);
                        HandState hand_1_state = neutral;
                        detectHandState(hand1_finger_distances,thumb,index,hand_1_state,8);
                        rot1 = movingAverage(hand1_rot,-calculateRotationAngle(thumb,index) + M_PI_2,5);
                        c1_x = movingAverage(hand1_center_x,(thumb.first+index.first) / 2,5);
                        c1_y = movingAverage(hand1_center_y,(thumb.second+index.second) / 2,5);
                        myvector->push_back(hand_1_state); //pointing finger

                        myvector->push_back(t1); //pointing finger
                        myvector->push_back(t2);
                        myvector->push_back(t3); //thumb finger
                        myvector->push_back(t4);
                        myvector->push_back(rot1*1000); //multiply to "cast" to int with 3 precision digits
                        myvector->push_back(c1_x);
                        myvector->push_back(c1_y);

                        condvar->notify_one();
                        count_hand_not_found_1 = 0;
                    
                }
                //Send two fingers info, not just centroid
                else if (GoodContours.size() == num_hands_bycolor*2 && fingers_info_to_send.find("centroid") == std::string::npos){
                    std::vector<cv::Point> centers; 
                    for (int i=0; i<GoodContours.size(); i++){
                        cv::Moments M = cv::moments(GoodContours[i]);
                        cv::Point center(M.m10/M.m00, M.m01/M.m00);
                        centers.push_back(center);
                    }
                    hand_1_closed = false;
                    hand_1_active = true;
                    
                    float center_distances = sqrt(pow(centers[0].x-centers[1].x,2)+pow(centers[0].y-centers[1].y,2));
                    //std::cout<<"center_distances: "<<center_distances<<std::endl;
                    if (center_distances > 10 && center_distances<500){
                        myvector->push_back(1); //num_hands
                        //HAND 1
                        myvector->push_back(int(hand_1_closed)); //hand 1 closed
                        myvector->push_back(int(hand_1_active)); //hand 1 closed
                        // t1 = movingAverage(hand1_l1,keypoints[0].pt.x * SCALE);
                        // t2 = movingAverage(hand1_l2,keypoints[0].pt.y* SCALE);
                        // t3 = movingAverage(hand1_l3,keypoints[1].pt.x* SCALE,10);
                        // t4 = movingAverage(hand1_l4,keypoints[1].pt.y* SCALE,10);
                        t3 = movingAverage(hand1_l1,centers[0].x * scale_dueto_crop * SCALE,2);
                        t4 = movingAverage(hand1_l2,centers[0].y * scale_dueto_crop * SCALE,2);
                        t1 = movingAverage(hand1_l3,centers[1].x * scale_dueto_crop * SCALE,2);
                        t2 = movingAverage(hand1_l4,centers[1].y * scale_dueto_crop * SCALE,2);
                        
                        //Get hand state
                        std::pair index = std::make_pair(t1, t2);
                        std::pair thumb = std::make_pair(t3, t4);
                        HandState hand_1_state = neutral;
                        detectHandState(hand1_finger_distances,thumb,index,hand_1_state,8);
                        rot1 = movingAverage(hand1_rot,-calculateRotationAngle(thumb,index) + M_PI_2,5);
                        c1_x = movingAverage(hand1_center_x,(thumb.first+index.first) / 2,5);
                        c1_y = movingAverage(hand1_center_y,(thumb.second+index.second) / 2,5);
                        myvector->push_back(hand_1_state); //pointing finger

                        myvector->push_back(t1); //pointing finger
                        myvector->push_back(t2);
                        myvector->push_back(t3); //thumb finger
                        myvector->push_back(t4);
                        myvector->push_back(rot1*1000); //multiply to "cast" to int with 3 precision digits
                        myvector->push_back(c1_x);
                        myvector->push_back(c1_y);
                        std::cout<<"Hand state: "<<hand_1_state<<" t4: "<<t4<<" rot: "<<rot1<<std::endl;

                        condvar->notify_one();
                        count_hand_not_found_1 = 0;
                    }
                }
                else{
                    count_hand_not_found_1 ++;
                    count_hand_not_found_2 ++;
                }
                // Clear the gesture array.
                if (count_hand_not_found_1>COUNT_NOT_HANDS){
                    hand1_l1.clear(); hand1_l2.clear(); hand1_l3.clear(); hand1_l4.clear(); hand1_rot.clear();
                }
                if (count_hand_not_found_2>COUNT_NOT_HANDS){
                    hand2_l1.clear(); hand2_l2.clear(); hand2_l3.clear(); hand2_l4.clear();
                }   
            }
            // Draw detected blobs as red circles.
            // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
            cv::Mat im_with_keypoints;

            //cv::drawKeypoints( image_cv, keypoints, im_with_keypoints, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
            
            // Show blobs
            //cv::imshow("keypoints", im_with_keypoints );
            if (show_image){
                cv::imshow("roi_img", roi_img);
                cv::imshow("Contours", contourImage);
                cv::imshow("Thresholded Image", imgThresholded); //show the thresholded image
            }
            

            delta_ticks = clock() - current_ticks; //the time, in ms, that took to render the scene
            if(delta_ticks > 0)
                fps = CLOCKS_PER_SEC / delta_ticks;
            if(fps){
                std::cout << "  FPS:"<<fps << std::endl;
            }
            frame_count+=1;
            if (frame_count>1000){
                frame_count=0;
            }
            char c = cv::waitKey(1);
            if(c == 27) {
                std::cout << "ESC pressed" << std::endl;
                grab_frames = false;
            }
            else if(c=='q'){
                show_image = false;
                cv::destroyAllWindows();
            }
        //LOG(INFO) << fps;

        }
    }
    // free allocated memory before closing the ZED
    //point_cloud.free();
    // close the ZED
    zed.close();
    LOG(INFO) << "Shutting down.";
    if (writer.isOpened()) writer.release();
    MP_RETURN_IF_ERROR(graph.CloseInputStream(kInputStream));
    return graph.WaitUntilDone();
  }
  else if(camera_type.find("webcam") != std::string::npos){
    std::cout<<"Let's use the webcam"<<std::endl;
    cv::VideoCapture capture;
    const bool load_video = !absl::GetFlag(FLAGS_input_video_path).empty();
    if (load_video) {
      capture.open(absl::GetFlag(FLAGS_input_video_path));
    } else {
      capture.open(0);
    }
    RET_CHECK(capture.isOpened());
    
    cv::VideoWriter writer;
    const bool save_video = !absl::GetFlag(FLAGS_output_video_path).empty();
    //   if (!save_video) {
    //     cv::namedWindow(kWindowName, /*flags=WINDOW_AUTOSIZE*/ 1);
    // #if (CV_MAJOR_VERSION >= 3) && (CV_MINOR_VERSION >= 2)
    //     capture.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    //     capture.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    //     capture.set(cv::CAP_PROP_FPS, 30);
    // #endif
    //   }

    LOG(INFO) << "Start running the calculator graph.";
    ASSIGN_OR_RETURN(mediapipe::OutputStreamPoller pPoller_landmarks,
                        graph.AddOutputStreamPoller(kOutputLandmarks));
    MP_RETURN_IF_ERROR(graph.StartRun({}));

    LOG(INFO) << "Start grabbing and processing frames.";
    bool grab_frames = true;
    sl::Mat image_zed;
    int count_active_1 = 0;
    int count_active_2 = 0;
    int count_closed_1 = 0;
    int count_closed_2 = 0;
    double distance_1 = 300.0;
    double distance_2 = 300.0;
    int count_hand_not_found_1 = 0, count_hand_not_found_2 = 0;
    double t1, t2, t3, t4;
    double t5, t6, t7, t8;
    std::vector<double> hand1_l1, hand1_l2, hand1_l3, hand1_l4;
    std::vector<double> hand2_l1, hand2_l2, hand2_l3, hand2_l4;
    while (grab_frames) {
        // Main Loop
       
        //viewer.updatePointCloud(point_cloud);
        current_ticks = clock();
        // Capture opencv camera or video frame.
        cv::Mat camera_frame_raw;
        capture >> camera_frame_raw;
        if (camera_frame_raw.empty()) {
        if (!load_video) {
            LOG(INFO) << "Ignore empty frames from camera.";
            continue;
        }
        LOG(INFO) << "Empty frame, end of video reached.";
        break;
        }
            cv::Mat camera_frame;
            cv::cvtColor(camera_frame_raw, camera_frame, cv::COLOR_BGR2RGBA);

            //cv::Mat image_ocv = slMat2cvMat(image_zed);
            //cv::flip(point_cloud, point_cloud, /*flipcode=HORIZONTAL*/ 1);
            //std::cout<<camera_frame.cols<<"  "<<camera_frame.rows<<std::endl;
            cv::flip(camera_frame, camera_frame, /*flipcode=HORIZONTAL*/ 1);
            //if (!load_video) {
            // }

            // Wrap Mat into an ImageFrame.
            auto input_frame = absl::make_unique<mediapipe::ImageFrame>(
                mediapipe::ImageFormat::SRGBA, camera_frame.cols, camera_frame.rows,
                mediapipe::ImageFrame::kGlDefaultAlignmentBoundary);
            cv::Mat input_frame_mat = mediapipe::formats::MatView(input_frame.get());
            camera_frame.copyTo(input_frame_mat);

            // Prepare and add graph input packet.
            size_t frame_timestamp_us =
                (double)cv::getTickCount() / (double)cv::getTickFrequency() * 1e6;
            MP_RETURN_IF_ERROR(
                gpu_helper.RunInGlContext([&input_frame, &frame_timestamp_us, &graph,
                                        &gpu_helper]() -> absl::Status {
                // Convert ImageFrame to GpuBuffer.
                auto texture = gpu_helper.CreateSourceTexture(*input_frame.get());
                auto gpu_frame = texture.GetFrame<mediapipe::GpuBuffer>();
                glFlush();
                texture.Release();
                // Send GPU image packet into the graph.
                MP_RETURN_IF_ERROR(graph.AddPacketToInputStream(
                    kInputStream, mediapipe::Adopt(gpu_frame.release())
                                        .At(mediapipe::Timestamp(frame_timestamp_us))));
                return absl::OkStatus();
                }));

            // Get the graph result packet, or stop if that fails.
            // mediapipe::Packet packet;
            // if (!poller.Next(&packet)) break;
            // std::unique_ptr<mediapipe::ImageFrame> output_frame;
            // mediapipe::Packet multi_hand_landmarks_packet;
            //Landmark reading

            mediapipe::Packet packet_landmarks;
            std::vector<float> hand_landmarks;
            hand_landmarks.clear();
            //printf("poller is not null.\n");
            if(pPoller_landmarks.QueueSize() > 0){
                if (pPoller_landmarks.Next(&packet_landmarks)){
                    std::vector<mediapipe::NormalizedLandmarkList> output_landmarks =
                    packet_landmarks.Get<std::vector<mediapipe::NormalizedLandmarkList>>();
                    for (int m = 0; m < output_landmarks.size(); ++m){
                        mediapipe::NormalizedLandmarkList single_hand_NormalizedLandmarkList = output_landmarks[m];
                        for (int i = 0; i < single_hand_NormalizedLandmarkList.landmark_size(); ++i){
                            const mediapipe::NormalizedLandmark landmark = single_hand_NormalizedLandmarkList.landmark(i);
                            
                            int x = landmark.x() * camera_frame.cols;
                            int y = landmark.y() * camera_frame.rows;
                            //int x = landmark.x();
                            //int y = landmark.y();
                            hand_landmarks.push_back(x);
                            hand_landmarks.push_back(y);
                        }
                    }
                    bool hand_1_closed = false;
                    bool hand_2_closed = false;
                    bool hand_1_active = false;
                    bool hand_2_active = false;
                    float multiplier = 1.0;
                    if (hand_landmarks.size()==42){
                        //printf("hand 1 is closed.\n");
                        hand_1_closed = isHandClosed(hand_landmarks,0,1,count_closed_1);
                        //hand_1_active = isHandActive(hand_landmarks,point_cloud,0,1,count_active_1,multiplier,distance_1);
                        hand_1_active = true;
                        std::cout<<" "<<hand_1_closed<<"  "<<hand_1_active<<std::endl;
                    }
                    else if(hand_landmarks.size()==84){
                        //printf("hand 2 is closed.\n");
                        hand_1_closed = isHandClosed(hand_landmarks,0,2,count_closed_1);
                        hand_1_active = true;
                        hand_2_closed = isHandClosed(hand_landmarks, 42,2,count_closed_2);
                        hand_2_active = true;
                        std::cout<<hand_1_closed<<" "<<hand_1_active<<" "<<hand_2_closed<<"  "<<hand_2_active;
                    }
                    //printf("hand points size = %d.\n", hand_landmarks.size());
                    // Lock the mutex
                    std::unique_lock<Mutex> lock(*mutex);
                    // Write the data to shared memory
                    myvector->clear();
                    //for (int i = 0; i < 2; i++) {
                    if (hand_landmarks.size()==42){
                        if (count_hand_not_found_1 == 0 && count_hand_not_found_2 == 0){
                            hand1_l1.clear(); hand1_l2.clear(); hand1_l3.clear(); hand1_l4.clear();
                        } 
                        if (distance_1<1000 && distance_1>0){
                            //std::cout<<"pushing data 1 = "<<distance_1<<std::endl;
                            myvector->push_back(1); //num_hands
                            //HAND 1
                            myvector->push_back(int(hand_1_closed)); //hand 1 closed
                            myvector->push_back(int(hand_1_active)); //hand 1 closed
                            t1 = movingAverage(hand1_l1,hand_landmarks[16]* SCALE);
                            t2 = movingAverage(hand1_l2,hand_landmarks[17]* SCALE);
                            t3 = movingAverage(hand1_l3,hand_landmarks[8]* SCALE);
                            t4 = movingAverage(hand1_l4,hand_landmarks[9]* SCALE);
                            myvector->push_back(t1); //pointing finger
                            myvector->push_back(t2);
                            myvector->push_back(t3); //thumb finger
                            myvector->push_back(t4);
                            condvar->notify_one();
                            count_hand_not_found_1 = 0;
                            count_hand_not_found_2 ++;
                        }
                        else{
                            count_hand_not_found_1 ++;
                            count_hand_not_found_2 ++;
                        }
                    }
                    else if(hand_landmarks.size()==84){
                        if (distance_1<1000 && distance_1>0 && distance_2<1000 && distance_2>0){
                            myvector->push_back(2); //num_hands
                            //HAND 1
                            myvector->push_back(int(hand_2_closed)); //hand 2 closed
                            myvector->push_back(int(hand_2_active)); //hand 1 closed
                            t5 = movingAverage(hand1_l1,hand_landmarks[58]* SCALE); //pointing finger
                            t6 = movingAverage(hand1_l2,hand_landmarks[59]* SCALE);
                            t7 = movingAverage(hand1_l3,hand_landmarks[50]* SCALE); //thumb finger
                            t8 = movingAverage(hand1_l4,hand_landmarks[51]* SCALE);
                            myvector->push_back(t5); //pointing finger
                            myvector->push_back(t6);
                            myvector->push_back(t7); //thumb finger
                            myvector->push_back(t8);
                            //HAND 2
                            myvector->push_back(int(hand_1_closed)); //hand 1 closed
                            myvector->push_back(int(hand_1_active)); //hand 1 closed
                            t1 = movingAverage(hand2_l1,hand_landmarks[16]* SCALE);  //pointing finger  
                            t2 = movingAverage(hand2_l2,hand_landmarks[17]* SCALE);
                            t3 = movingAverage(hand2_l3,hand_landmarks[8]* SCALE); //thumb finger
                            t4 = movingAverage(hand2_l4,hand_landmarks[9]* SCALE);
                            myvector->push_back(t1); //pointing finger
                            myvector->push_back(t2);
                            myvector->push_back(t3); //thumb finger
                            myvector->push_back(t4);

                            condvar->notify_one();
                            count_hand_not_found_1 = 0;
                            count_hand_not_found_2 = 0;
                        }
                        // else if (distance_1<1000 && distance_1>0){
                        //     myvector->push_back(1); //num_hands
                        //     myvector->push_back(int(hand_1_closed)); //hand 1 closed
                        //     myvector->push_back(int(hand_1_active)); //hand 1 closed
                        //     t1 = movingAverage(hand1_l1,hand_landmarks[16]* SCALE);  //pointing finger  
                        //     t2 = movingAverage(hand1_l2,hand_landmarks[17]* SCALE);
                        //     t3 = movingAverage(hand1_l3,hand_landmarks[8]* SCALE); //thumb finger
                        //     t4 = movingAverage(hand1_l4,hand_landmarks[9]* SCALE);
                        //     myvector->push_back(t1); //pointing finger
                        //     myvector->push_back(t2);
                        //     myvector->push_back(t3); //thumb finger
                        //     myvector->push_back(t4);
                        //     condvar->notify_one();
                        //     count_hand_not_found_1 = 0;
                        //     count_hand_not_found_2 ++;

                        // }
                        // else if (distance_2<1000 && distance_2>0){
                        //     myvector->push_back(1); //num_hands
                        //     t5 = movingAverage(hand2_l1,hand_landmarks[58]* SCALE); //pointing finger
                        //     t6 = movingAverage(hand2_l2,hand_landmarks[59]* SCALE);
                        //     t7 = movingAverage(hand2_l3,hand_landmarks[50]* SCALE); //thumb finger
                        //     t8 = movingAverage(hand2_l4,hand_landmarks[51]* SCALE);
                        //     myvector->push_back(t5); //pointing finger
                        //     myvector->push_back(t6);
                        //     myvector->push_back(t7); //thumb finger
                        //     myvector->push_back(t8);
                        //     condvar->notify_one();
                        //     count_hand_not_found_2 = 0;
                        //     count_hand_not_found_1 ++;
                        // }
                    }
                    // Notify the condition variable
                }
            }
            else{
                count_hand_not_found_1 ++;
                count_hand_not_found_2 ++;
            }
            // Clear the gesture array.
            if (count_hand_not_found_1>COUNT_NOT_HANDS){
                hand1_l1.clear(); hand1_l2.clear(); hand1_l3.clear(); hand1_l4.clear();
            }
            if (count_hand_not_found_2>COUNT_NOT_HANDS){
                hand2_l1.clear(); hand2_l2.clear(); hand2_l3.clear(); hand2_l4.clear();
            }
            //std::cout<<" "<<count_hand_not_found_1<<" "<<count_hand_not_found_2<<" "<<hand1_l1.size()<<" "<<hand2_l1.size()<<std::endl;
            // Press any key to exit.
            const int pressed_key = cv::waitKey(5);
            if (pressed_key >= 0 && pressed_key != 255) grab_frames = false;

            delta_ticks = clock() - current_ticks; //the time, in ms, that took to render the scene
            if(delta_ticks > 0)
                fps = CLOCKS_PER_SEC / delta_ticks;
            std::cout << "FPS:"<<fps << std::endl;
        }
        //LOG(INFO) << fps;
        LOG(INFO) << "Shutting down.";
        if (writer.isOpened()) writer.release();
        MP_RETURN_IF_ERROR(graph.CloseInputStream(kInputStream));
        return graph.WaitUntilDone();
    }
  
}

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    absl::ParseCommandLine(argc, argv);
    absl::Status run_status = RunMPPGraph(argc,argv);
    if (!run_status.ok()) {
        LOG(ERROR) << "Failed to run the graph: " << run_status.message();
        return EXIT_FAILURE;
    } else {
        LOG(INFO) << "Success!";
    }

    return EXIT_SUCCESS;
}

void parseArgs(int argc, char **argv, sl::InitParameters& param) {
    if (argc > 1 && std::string(argv[1]).find(".svo") != std::string::npos) {
        // SVO input mode
        param.input.setFromSVOFile(argv[1]);
        std::cout << "[Sample] Using SVO File input: " << argv[1] << std::endl;
    } else if (argc > 1 && std::string(argv[1]).find(".svo") == std::string::npos) {
        std::string arg = std::string(argv[1]);
        unsigned int a, b, c, d, port;
        if (sscanf(arg.c_str(), "%u.%u.%u.%u:%d", &a, &b, &c, &d, &port) == 5) {
            // Stream input mode - IP + port
            std::string ip_adress = std::to_string(a) + "." + std::to_string(b) + "." + std::to_string(c) + "." + std::to_string(d);
            param.input.setFromStream(sl::String(ip_adress.c_str()), port);
            std::cout << "[Sample] Using Stream input, IP : " << ip_adress << ", port : " << port << std::endl;
        } else if (sscanf(arg.c_str(), "%u.%u.%u.%u", &a, &b, &c, &d) == 4) {
            // Stream input mode - IP only
            param.input.setFromStream(sl::String(argv[1]));
            std::cout << "[Sample] Using Stream input, IP : " << argv[1] << std::endl;
        } else if (arg.find("HD2K") != std::string::npos) {
            param.camera_resolution = sl::RESOLUTION::HD2K;
            std::cout << "[Sample] Using Camera in resolution HD2K" << std::endl;
        } else if (arg.find("HD1080") != std::string::npos) {
            param.camera_resolution = sl::RESOLUTION::HD1080;
            std::cout << "[Sample] Using Camera in resolution HD1080" << std::endl;
        } else if (arg.find("HD720") != std::string::npos) {
            param.camera_resolution = sl::RESOLUTION::HD720;
            std::cout << "[Sample] Using Camera in resolution HD720" << std::endl;
        } else if (arg.find("VGA") != std::string::npos) {
            param.camera_resolution = sl::RESOLUTION::VGA;
            std::cout << "[Sample] Using Camera in resolution VGA" << std::endl;
        } else if (arg.find("webcam") != std::string::npos) {
            //param.camera_resolution = sl::RESOLUTION::VGA;
            std::cout << "[Sample] Using Camera in resolution VGA" << std::endl;
        }
    } else {
        // Default
    }
}
