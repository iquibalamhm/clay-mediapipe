// Copyright 2019 The MediaPipe Authors.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// An example of sending OpenCV webcam frames into a MediaPipe graph.
// This example requires a linux computer and a GPU with EGL support drivers.
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

constexpr char kInputStream[] = "input_video";
//constexpr char kWindowName[] = "MediaPipe";
constexpr char kOutputLandmarks[] = "hand_landmarks";

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

bool isHandClosed(const std::vector<float>& landmarks, int start_index,int num_hands){
  if (landmarks.size() != NUM_LANDMARKS * 2 * num_hands) {
    std::cerr << "Invalid number of landmarks. Expected " << NUM_LANDMARKS * 2 *num_hands
              << " but got " << landmarks.size() << std::endl;
    return false;
  }

  // Here you can use the 21 landmarks to calculate the distance between thumb and other fingers
  // based on the position of the landmarks, and determine if the hand is closed or not.
  // ...

  // For example, a simple heuristic to detect if a hand is closed is to check if the distance
  // between the thumb and the pinky finger is small enough, which would indicate that the hand
  // is closed.

  // Calculate the distance between the thumb and pinky finger.
  float thumb_x = landmarks[start_index+8];
  float thumb_y = landmarks[start_index + 9];
  float pinky_x = landmarks[start_index + 40];
  float pinky_y = landmarks[start_index + 41];
  float distance = std::sqrt((thumb_x - pinky_x) * (thumb_x - pinky_x) +
                             (thumb_y - pinky_y) * (thumb_y - pinky_y));

  // If the distance is less than a certain threshold, the hand is considered closed.
  if (distance < 60.0) {
    return true;
  } else {
    return false;
  }
}

absl::Status RunMPPGraph() {
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
  while (grab_frames) {

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
    //std::cout<<camera_frame.cols<<"  "<<camera_frame.rows<<std::endl;
    if (!load_video) {
      cv::flip(camera_frame, camera_frame, /*flipcode=HORIZONTAL*/ 1);
    }

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
        if (hand_landmarks.size()==42){
          //printf("hand 1 is closed.\n");
          hand_1_closed = isHandClosed(hand_landmarks,0,1);  
          std::cout<<"hand 1 closed: "<<hand_1_closed<<std::endl;
        }
        else if(hand_landmarks.size()==84){
          //printf("hand 2 is closed.\n");
          hand_1_closed = isHandClosed(hand_landmarks,0,2);
          hand_2_closed = isHandClosed(hand_landmarks, 42,2);
          std::cout<<"hand 1 closed: "<<hand_1_closed<<"hand 2 closed: "<<hand_2_closed<<std::endl;
        }
        //printf("hand points size = %d.\n", hand_landmarks.size());
        // Lock the mutex
        std::unique_lock<Mutex> lock(*mutex);
        // Write the data to shared memory
        myvector->clear();
        //for (int i = 0; i < 2; i++) {
        if (hand_landmarks.size()==42){
          myvector->push_back(1); //num_hands
          myvector->push_back(int(hand_1_closed)); //hand 1 closed
          myvector->push_back(hand_landmarks[16]); //pointing finger
          myvector->push_back(hand_landmarks[17]);
          myvector->push_back(hand_landmarks[8]); //thumb finger
          myvector->push_back(hand_landmarks[9]);
        }
        else if(hand_landmarks.size()==84){
          myvector->push_back(2); //num_hands
          //HAND 1
          myvector->push_back(int(hand_1_closed)); //hand 1 closed
          myvector->push_back(hand_landmarks[16]); //pointing finger
          myvector->push_back(hand_landmarks[17]);
          myvector->push_back(hand_landmarks[8]); //thumb finger
          myvector->push_back(hand_landmarks[9]);
          
          //HAND 2
          myvector->push_back(int(hand_2_closed)); //hand 2 closed
          myvector->push_back(hand_landmarks[58]); //pointing finger
          myvector->push_back(hand_landmarks[59]);
          myvector->push_back(hand_landmarks[50]); //thumb finger
          myvector->push_back(hand_landmarks[51]);
        }
        //}
        // for (int i = 0; i < 4; i++) {
        //     myvector->push_back(rand() % 100);
        // }

        // Notify the condition variable
        condvar->notify_one();
      }
    }

    // Convert GpuBuffer to ImageFrame.
    // MP_RETURN_IF_ERROR(gpu_helper.RunInGlContext(
    //     [&packet, &output_frame, &gpu_helper]() -> absl::Status {
    //       auto& gpu_frame = packet.Get<mediapipe::GpuBuffer>();
    //       auto texture = gpu_helper.CreateSourceTexture(gpu_frame);
    //       output_frame = absl::make_unique<mediapipe::ImageFrame>(
    //           mediapipe::ImageFormatForGpuBufferFormat(gpu_frame.format()),
    //           gpu_frame.width(), gpu_frame.height(),
    //           mediapipe::ImageFrame::kGlDefaultAlignmentBoundary);
    //       gpu_helper.BindFramebuffer(texture);
    //       const auto info = mediapipe::GlTextureInfoForGpuBufferFormat(
    //           gpu_frame.format(), 0, gpu_helper.GetGlVersion());
    //       glReadPixels(0, 0, texture.width(), texture.height(), info.gl_format,
    //                    info.gl_type, output_frame->MutablePixelData());
    //       glFlush();
    //       texture.Release();
    //       return absl::OkStatus();
    //     }));

    // // Convert back to opencv for display or saving.
    // cv::Mat output_frame_mat = mediapipe::formats::MatView(output_frame.get());
    // if (output_frame_mat.channels() == 4)
    //   cv::cvtColor(output_frame_mat, output_frame_mat, cv::COLOR_RGBA2BGR);
    // else
    //   cv::cvtColor(output_frame_mat, output_frame_mat, cv::COLOR_RGB2BGR);
    // if (save_video) {
    //   if (!writer.isOpened()) {
    //     LOG(INFO) << "Prepare video writer.";
    //     writer.open(absl::GetFlag(FLAGS_output_video_path),
    //                 mediapipe::fourcc('a', 'v', 'c', '1'),  // .mp4
    //                 capture.get(cv::CAP_PROP_FPS), output_frame_mat.size());
    //     RET_CHECK(writer.isOpened());
    //   }
    //   writer.write(output_frame_mat);
    // } else {
    //   cv::imshow(kWindowName, output_frame_mat);
    //   // Press any key to exit.
    //   const int pressed_key = cv::waitKey(5);
    //   if (pressed_key >= 0 && pressed_key != 255) grab_frames = false;
    // }

    // Press any key to exit.
    const int pressed_key = cv::waitKey(5);
    if (pressed_key >= 0 && pressed_key != 255) grab_frames = false;

    delta_ticks = clock() - current_ticks; //the time, in ms, that took to render the scene
    if(delta_ticks > 0)
        fps = CLOCKS_PER_SEC / delta_ticks;
    std::cout << "FPS:"<<fps << std::endl;

    //LOG(INFO) << fps;
  }

  LOG(INFO) << "Shutting down.";
  if (writer.isOpened()) writer.release();
  MP_RETURN_IF_ERROR(graph.CloseInputStream(kInputStream));
  return graph.WaitUntilDone();
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  absl::ParseCommandLine(argc, argv);
  absl::Status run_status = RunMPPGraph();
  if (!run_status.ok()) {
    LOG(ERROR) << "Failed to run the graph: " << run_status.message();
    return EXIT_FAILURE;
  } else {
    LOG(INFO) << "Success!";
  }
  return EXIT_SUCCESS;
}
