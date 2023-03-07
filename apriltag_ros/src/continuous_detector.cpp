/**
 * Copyright (c) 2017, California Institute of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the California Institute of
 * Technology.
 */

#include "apriltag_ros/continuous_detector.h"

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(apriltag_ros::ContinuousDetector, nodelet::Nodelet);

namespace apriltag_ros
{
void ContinuousDetector::onInit ()
{
  ros::NodeHandle& nh = getNodeHandle();
  ros::NodeHandle& pnh = getPrivateNodeHandle();

  tag_detector_ = std::shared_ptr<TagDetector>(new TagDetector(pnh));
  draw_tag_detections_image_ = getAprilTagOption<bool>(pnh, 
      "publish_tag_detections_image", false);
  it_ = std::shared_ptr<image_transport::ImageTransport>(
      new image_transport::ImageTransport(nh));

  std::string transport_hint;
  pnh.param<std::string>("transport_hint", transport_hint, "raw");

  single_shot_detection_ = getAprilTagOption<bool>(pnh,"single_shot_detection",false);

  int queue_size;
    pnh.param<int>("queue_size", queue_size, 1);
    camera_image_subscriber_ =
        it_->subscribeCamera("image_rect", queue_size,
                            &ContinuousDetector::imageCallback, this,
                            image_transport::TransportHints(transport_hint));

  if (!single_shot_detection_)
  {
    ROS_INFO("Running in continuous mode.");
  }
  else{
    ROS_INFO("Running in single shot detection mode.");
  }
  
  tag_detections_publisher_ =
      nh.advertise<AprilTagDetectionArray>("tag_detections", 1);
  if (draw_tag_detections_image_)
  {
    tag_detections_image_publisher_ = it_->advertise("tag_detections_image", 1);
  }

  single_shot_service_ = nh.advertiseService("single_shot_detect_tags", &ContinuousDetector::singleShotService, this);
}

void ContinuousDetector::imageCallback (
    const sensor_msgs::ImageConstPtr& image_rect,
    const sensor_msgs::CameraInfoConstPtr& camera_info)
{
  // Lazy updates:
  // When there are no subscribers _and_ when tf is not published,
  // skip detection.
  if (tag_detections_publisher_.getNumSubscribers() == 0 &&
      tag_detections_image_publisher_.getNumSubscribers() == 0 &&
      !tag_detector_->get_publish_tf())
  {
    // ROS_INFO_STREAM("No subscribers and no tf publishing, skip processing.");
    return;
  }

  // Convert ROS's sensor_msgs::Image to cv_bridge::CvImagePtr in order to run
  // AprilTag 2 on the iamge
  try
  {
    cv_image_ = cv_bridge::toCvCopy(image_rect, image_rect->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Publish detected tags in the image by AprilTag 2
  if (single_shot_detection_)
  {
    detections_ = tag_detector_->detectTags(cv_image_,camera_info);
    tag_detections_publisher_.publish(detections_);
    have_detection_ = true;
  }else{
    tag_detections_publisher_.publish(
        tag_detector_->detectTags(cv_image_,camera_info));
  }
  // Publish the camera image overlaid by outlines of the detected tags and
  // their payload values
  if (draw_tag_detections_image_)
  {
    tag_detector_->drawDetections(cv_image_);
    tag_detections_image_publisher_.publish(cv_image_->toImageMsg());
  }

}

bool ContinuousDetector::singleShotService(
    apriltag_ros::SingleShotRequest& request,
    apriltag_ros::SingleShotResponse& response)
{ //TODO Does it make sense to check the timestamp on detections_ to make sure it's not too old?
  //TODO This is probably not the right if condition. Need a way to make sure that the detections object is valid. Could be trash in which case we should fail the service call.
  //TODO does it make sense for the image subscriber to check for blank/black images? maybe that can be treated as a failure rather than a "no tags detected" mode.
  if (camera_image_subscriber_ && have_detection_){
    camera_image_subscriber_ = it_->subscribeCamera("image_rect", 1, &ContinuousDetector::imageCallback, this);
    response.success = true;
    response.message = "Single shot tag detection activated.";
    response.num_detections = detections_.detections.size();
    response.any_detections = response.num_detections>0;
    response.tag_detections = detections_;
    detections_.detections.clear(); // TODO is this memory-leak safe/free? if detections contain (stupid) pointers to dynamically allocated memory, then that's a leak
    detections_.header = std_msgs::Header();
    have_detection_ = false; //this probably removes the need to reset the detection object
  }
  else
  {
    response.success = false;
    if (!single_shot_detection_){
      response.message = "Not in single shot detection mode.";
      return false;
    }else if (!have_detection_){
      response.message = "No valid detection data available.";
      return false;
    }else{
      response.message = "Unclear what went wrong.";
      return false;
    }
  }
  return true;
}

} // namespace apriltag_ros
