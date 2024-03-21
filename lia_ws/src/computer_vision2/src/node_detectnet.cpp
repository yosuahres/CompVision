#include <jetson-inference/detectNet.h>
#include <jetson-utils/videoOutput.h>

#include <unordered_map>

#include "image_converter.h"
#include "ros_compat.h"

// ros::Subscriber* sub = NULL;

// globals
videoOutput *output = NULL;
detectNet *net = NULL;
uint32_t overlay_flags = detectNet::OVERLAY_NONE;

imageConverter *input_cvt = NULL;
imageConverter *overlay_cvt = NULL;

Publisher<sensor_msgs::Image> overlay_pub = NULL;

// triggered when a new subscriber connected

// publish overlay image
bool publish_overlay(detectNet::Detection *detections, int numDetections)
{
  std::cout << "publisher called" << std::endl;
  // get the image dimensions
  const uint32_t width = input_cvt->GetWidth();
  const uint32_t height = input_cvt->GetHeight();

  // assure correct image size
  if (!overlay_cvt->Resize(width, height, imageConverter::ROSOutputFormat))
  {
  }
  // return false;

  // generate the overlay
  if (!net->Overlay(input_cvt->ImageGPU(), overlay_cvt->ImageGPU(), width, height, imageConverter::InternalFormat, detections, numDetections,
                    overlay_flags))
  {
    // return false;
  }

  // populate the message
  sensor_msgs::Image msg;

  if (!overlay_cvt->Convert(msg, imageConverter::ROSOutputFormat))
  {
  }
  // return false;

  // populate timestamp in header field
  // msg.header.stamp = ROS_TIME_NOW();

  // publish the message

  // overlay_pub->publish(msg);
  ROS_DEBUG("publishing %ux%u overlay image", width, height);

  output->Render(overlay_cvt->ImageGPU(), overlay_cvt->GetWidth(), overlay_cvt->GetHeight());
}

// input image subscriber callback
void img_callback(const sensor_msgs::Image::ConstPtr &input)
{
  std::cout << "subscriber called" << std::endl;
  // convert the image to reside on GPU
  if (!input_cvt || !input_cvt->Convert(input))
  {
    ROS_INFO("failed to convert %ux%u %s image", input->width, input->height, input->encoding.c_str());
    return;
  }

  // classify the image
  detectNet::Detection *detections = NULL;

  const int numDetections = net->Detect(input_cvt->ImageGPU(), input_cvt->GetWidth(), input_cvt->GetHeight(), &detections, detectNet::OVERLAY_NONE);

  // verify success
  if (numDetections < 0)
  {
    ROS_ERROR("failed to run object detection on %ux%u image", input->width, input->height);
    return;
  }

  // if objects were detected, send out message
  if (numDetections > 0)
  {
    ROS_INFO("detected %i objects in %ux%u image", numDetections, input->width, input->height);

    // create a detection for each bounding box

    for (int n = 0; n < numDetections; n++)
    {
      detectNet::Detection *det = detections + n;

      ROS_INFO("object %i class #%u (%s)  confidence=%f", n, det->ClassID, net->GetClassDesc(det->ClassID), det->Confidence);
      ROS_INFO("object %i bounding box (%f, %f)  (%f, %f)  w=%f  h=%f", n, det->Left, det->Top, det->Right, det->Bottom, det->Width(), det->Height());

      // create a detection sub-message

      // publish the detection message
    }

    // generate the overlay (if there are subscribers)
    if (ROS_NUM_SUBSCRIBERS(overlay_pub) > 0)
    {
    }
  }
  publish_overlay(detections, numDetections);
}

int main(int argc, char **argv)
{
  // output = videoOutput::Create(resource_str.c_str(), video_options);
  // output = videoOutput::Create(NULL);
  output = videoOutput::Create("webrtc://@:8544/output");


  /*
   * create node instance
   */
  ROS_CREATE_NODE("detectnet");

  /*
   * retrieve parameters
   */
  std::string model_name = "ssd-mobilenet-v2";
  std::string model_path;
  std::string prototxt_path;
  std::string class_labels_path;

  std::string input_blob = DETECTNET_DEFAULT_INPUT;
  // *load object detection network
  std::string output_cvg = DETECTNET_DEFAULT_COVERAGE;
  std::string output_bbox = DETECTNET_DEFAULT_BBOX;
  std::string overlay_str = "box,labels,conf";

  float mean_pixel = 0.0f;
  float threshold = DETECTNET_DEFAULT_THRESHOLD;

  ROS_DECLARE_PARAMETER("model_name", model_name);
  ROS_DECLARE_PARAMETER("model_path", model_path);
  ROS_DECLARE_PARAMETER("prototxt_path", prototxt_path);
  ROS_DECLARE_PARAMETER("class_labels_path", class_labels_path);
  ROS_DECLARE_PARAMETER("input_blob", input_blob);
  ROS_DECLARE_PARAMETER("output_cvg", output_cvg);
  ROS_DECLARE_PARAMETER("output_bbox", output_bbox);
  ROS_DECLARE_PARAMETER("overlay_flags", overlay_str);
  ROS_DECLARE_PARAMETER("mean_pixel_value", mean_pixel);
  ROS_DECLARE_PARAMETER("threshold", threshold);

  /*
   * retrieve parameters
   */
  ROS_GET_PARAMETER("model_name", model_name);
  ROS_GET_PARAMETER("model_path", model_path);
  ROS_GET_PARAMETER("prototxt_path", prototxt_path);
  ROS_GET_PARAMETER("class_labels_path", class_labels_path);
  ROS_GET_PARAMETER("input_blob", input_blob);
  ROS_GET_PARAMETER("output_cvg", output_cvg);
  ROS_GET_PARAMETER("output_bbox", output_bbox);
  ROS_GET_PARAMETER("overlay_flags", overlay_str);
  ROS_GET_PARAMETER("mean_pixel_value", mean_pixel);
  ROS_GET_PARAMETER("threshold", threshold);

  overlay_flags = detectNet::OverlayFlagsFromStr(overlay_str.c_str());
  //   model_name = "/home/bagas/ProgramRobot/ROS/PROG_PHYNIX_ROS1_Main/devel/lib/computer_vision2/ssd-mobilenet.onnx";
  // model_path = "./devel/lib/computer_vision2/anam.onnx";
    // model_path = "./src/computer_vision2/src/anam.onnx";

  //   model_path = "./devel/lib/computer_vision2/ssd-mobilenet.onnx";

  class_labels_path = "./src/computer_vision2/src/labels.txt";
  mean_pixel = 0.f;
  threshold = 0.75f;
  input_blob = "input_0";
  //   output_blob = NULL;
  // *load object detection network
  output_cvg = "scores";
  output_bbox = "boxes";
  //   maxBatchSize = DEFAULT_MAX_BATCH_SIZE;

  /*
   * load object detection network
   */
  if (model_path.size() > 0)
  {
    // create network using custom model paths
    net = detectNet::Create(prototxt_path.c_str(), model_path.c_str(), mean_pixel, class_labels_path.c_str(), threshold, input_blob.c_str(),
                            output_cvg.c_str(), output_bbox.c_str());
  }
  else
  {
    // create network using the built-in model
    net = detectNet::Create(model_name.c_str());
  }

  if (!net)
  {
    ROS_ERROR("failed to load detectNet model");
    return 0;
  }

  /*
   * create the class labels parameter vector
   */
  //   std::hash<std::string> model_hasher;  // hash the model path to avoid collisions on the param server
  //   std::string model_hash_str = std::string(net->GetModelPath()) + std::string(net->GetClassPath());
  //   const size_t model_hash = model_hasher(model_hash_str);

  //   ROS_INFO("model hash => %zu", model_hash);
  //   ROS_INFO("hash string => %s", model_hash_str.c_str());

  // obtain the list of class descriptions
  //   std::vector<std::string> class_descriptions;
  //   const uint32_t num_classes = net->GetNumClasses();

  //   for (uint32_t n = 0; n < num_classes; n++) class_descriptions.push_back(net->GetClassDesc(n));

  // create the key on the param server
  //   std::string class_key = std::string("class_labels_") + std::to_string(model_hash);

  //   ROS_DECLARE_PARAMETER(class_key, class_descriptions);
  //   ROS_SET_PARAMETER(class_key, class_descriptions);

  // populate the vision info msg
  //   std::string node_namespace = ROS_GET_NAMESPACE();
  //   ROS_INFO("node namespace => %s", node_namespace.c_str());

  /*
   * create image converter objects
   */
  input_cvt = new imageConverter();
  overlay_cvt = new imageConverter();

  if (!input_cvt || !overlay_cvt)
  {
    ROS_ERROR("failed to create imageConverter objects");
    return 0;
  }

  /*
   * advertise publisher topics
   */
  ROS_CREATE_PUBLISHER(sensor_msgs::Image, "overlay", 2, overlay_pub);

  /*
   * subscribe to image topic
   */
  //   auto img_sub = ROS_CREATE_SUBSCRIBER(sensor_msgs::Image, "image_in", 5, img_callback);
  ros::Subscriber sub = nh.subscribe("image_in", 1000000, img_callback);

  /*
   * wait for messages
   */
  //   ROS_INFO("detectnet node initialized, waiting for messages");
  ROS_SPIN();

  /*
   * free resources
   */
  delete net;
  delete input_cvt;
  delete overlay_cvt;

  return 0;
}