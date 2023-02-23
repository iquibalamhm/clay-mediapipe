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
# define SCALE 2
# define THRESHOLD_CLOSED 24
# define THRESHOLD_ACTIVE_MILIMETERS 700
# define C1 100
# define C2 585
# define C3 75
# define C4 750
# define C5 (C4-(C3*C1)/C1)/(1-C3/C1)
# define C6 (C2-C5)/C1
# define min_count_out 10 
# define FOCAL_LENGTH 4
constexpr char kInputStream[] = "input_video";
//constexpr char kWindowName[] = "MediaPipe";
constexpr char kOutputLandmarks[] = "hand_landmarks";
int width = 672;
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
    return cv::Mat(input.getHeight(), input.getWidth(), getOCVtype(input.getDataType()), input.getPtr<sl::uchar1>(sl::MEM::CPU), input.getStepBytes(sl::MEM::CPU));
}

cv::Mat slMat2cvMatGPU(sl::Mat& input) {
    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), getOCVtype(input.getDataType()), input.getPtr<sl::uchar1>(sl::MEM::GPU), input.getStepBytes(sl::MEM::GPU));
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

bool isHandActive(const std::vector<float>& landmarks, sl::Mat& point_cloud,int start_index,int num_hands,int& curr_count, float& multiplier,double& distance){
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
            if (depth_value > 100 && depth_value < 2000){
                distances.push_back(depth_value);
            }
        }
    }
    // float depth_value=0;
    // point_cloud.getValue(300,200,&depth_value,sl::MEM::GPU);
    //std::cout<<std::endl;
    // distances.push_back(depth_value);
    
    float median_depth = median(distances);
    if (median_depth>100 & median_depth<585)
        multiplier = 1;
    else{
        
    }
    distance = median_depth;
    std::cout<<median_depth<<" ";
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
      shm_remove() { boost::interprocess::shared_memory_object::remove("SharedMemory"); }
      ~shm_remove(){ boost::interprocess::shared_memory_object::remove("SharedMemory"); }
   } remover;
    // Create the shared memory segment
    boost::interprocess::managed_shared_memory segment(boost::interprocess::create_only, "SharedMemory", 65536);

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
    std::string arg = std::string(argv[1]);
  if(arg.find("zed") != std::string::npos){
    # define use_zed


    // cv::VideoCapture capture;
    // const bool load_video = !absl::GetFlag(FLAGS_input_video_path).empty();
    // if (load_video) {
    //   capture.open(absl::GetFlag(FLAGS_input_video_path));
    // } else {
    //   capture.open(0);
    // }
    // RET_CHECK(capture.isOpened());
  }
  else{
    # define use_webcam
  }
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
    runParameters.confidence_threshold = 50;
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
    width = camera_config.resolution.width;
  while (grab_frames) {
    // Main Loop
    // Check that a new image is successfully acquired
    #ifdef use_zed
    #endif
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

        cv::Mat image_ocv = slMat2cvMat(image_zed);
        cv::cvtColor(image_ocv, camera_frame, cv::COLOR_BGR2RGBA);
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
                    hand_1_active = isHandActive(hand_landmarks,point_cloud,0,1,count_active_1,multiplier,distance_1);
                    std::cout<<" "<<hand_1_closed<<"  "<<hand_1_active<<std::endl;
                }
                else if(hand_landmarks.size()==84){
                    //printf("hand 2 is closed.\n");
                    hand_1_closed = isHandClosed(hand_landmarks,0,2,count_closed_1);
                    hand_1_active = isHandActive(hand_landmarks,point_cloud,0,2,count_active_1,  multiplier,distance_1);
                    hand_2_closed = isHandClosed(hand_landmarks, 42,2,count_closed_2);
                    hand_2_active = isHandActive(hand_landmarks,point_cloud,42,2,count_active_2,multiplier,distance_2);
                    std::cout<<hand_1_closed<<" "<<hand_1_active<<" "<<hand_2_closed<<"  "<<hand_2_active<<std::endl;
                }
                //printf("hand points size = %d.\n", hand_landmarks.size());
                // Lock the mutex
                std::unique_lock<Mutex> lock(*mutex);
                // Write the data to shared memory
                myvector->clear();
                //for (int i = 0; i < 2; i++) {
                if (hand_landmarks.size()==42){
                    if (distance_1<1000 && distance_1>0){
                        myvector->push_back(1); //num_hands
                        myvector->push_back(int(hand_1_closed)); //hand 1 closed
                        myvector->push_back(int(hand_1_active)); //hand 1 closed
                        myvector->push_back(hand_landmarks[16]* SCALE); //pointing finger
                        myvector->push_back(hand_landmarks[17]* SCALE);
                        myvector->push_back(hand_landmarks[8]* SCALE); //thumb finger
                        myvector->push_back(hand_landmarks[9]* SCALE);
                        condvar->notify_one();
                    }
                }
                else if(hand_landmarks.size()==84){
                    if (distance_1<1000 && distance_1>0 && distance_2<1000 && distance_2>0){
                        myvector->push_back(2); //num_hands
                        //HAND 1
                        myvector->push_back(int(hand_1_closed)); //hand 1 closed
                        myvector->push_back(int(hand_1_active)); //hand 1 closed
                        myvector->push_back(hand_landmarks[16]* SCALE); //pointing finger
                        myvector->push_back(hand_landmarks[17]* SCALE);
                        myvector->push_back(hand_landmarks[8]* SCALE); //thumb finger
                        myvector->push_back(hand_landmarks[9]* SCALE);

                        //HAND 2
                        myvector->push_back(int(hand_2_closed)); //hand 2 closed
                        myvector->push_back(int(hand_2_active)); //hand 1 closed

                        myvector->push_back(hand_landmarks[58]* SCALE); //pointing finger
                        myvector->push_back(hand_landmarks[59]* SCALE);
                        myvector->push_back(hand_landmarks[50]* SCALE); //thumb finger
                        myvector->push_back(hand_landmarks[51]* SCALE);
                        condvar->notify_one();
                    }
                    else if (distance_1<1000 && distance_1>0){
                        myvector->push_back(1); //num_hands
                        myvector->push_back(int(hand_1_closed)); //hand 1 closed
                        myvector->push_back(int(hand_1_active)); //hand 1 closed
                        myvector->push_back(hand_landmarks[16]* SCALE); //pointing finger
                        myvector->push_back(hand_landmarks[17]* SCALE);
                        myvector->push_back(hand_landmarks[8]* SCALE); //thumb finger
                        myvector->push_back(hand_landmarks[9]* SCALE);
                        condvar->notify_one();
                    }
                    else if (distance_2<1000 && distance_2>0){
                        myvector->push_back(1); //num_hands
                        myvector->push_back(int(hand_2_closed)); //hand 2 closed
                        myvector->push_back(int(hand_2_active)); //hand 1 closed
                        myvector->push_back(hand_landmarks[58]* SCALE); //pointing finger
                        myvector->push_back(hand_landmarks[59]* SCALE);
                        myvector->push_back(hand_landmarks[50]* SCALE); //thumb finger
                        myvector->push_back(hand_landmarks[51]* SCALE);
                        condvar->notify_one();}
                }
                // Notify the condition variable
            }
        }


        // Press any key to exit.
        const int pressed_key = cv::waitKey(5);
        if (pressed_key >= 0 && pressed_key != 255) grab_frames = false;

        delta_ticks = clock() - current_ticks; //the time, in ms, that took to render the scene
        if(delta_ticks > 0)
            fps = CLOCKS_PER_SEC / delta_ticks;
        //std::cout << "FPS:"<<fps << std::endl;
    }
    //LOG(INFO) << fps;
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
