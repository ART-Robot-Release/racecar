/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011 Willow Garage, Inc.
 *    Radu Bogdan Rusu <rusu@willowgarage.com>
 *    Suat Gedikli <gedikli@willowgarage.com>
 *    Patrick Mihelich <mihelich@willowgarage.com>
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef OPENNI_CAMERA_DRIVER_H
#define OPENNI_CAMERA_DRIVER_H

// ROS communication
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <boost/thread.hpp>

// Configuration
#include <camera_info_manager/camera_info_manager.h>
#include <dynamic_reconfigure/server.h>
#include <freenect_camera/FreenectConfig.h>

// freenect wrapper
//#include <freenect_camera/freenect_driver.hpp>
#include <freenect_camera/tilt_driver.hpp>
// diagnostics
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

namespace freenect_camera
{
  ////////////////////////////////////////////////////////////////////////////////////////////
  class DriverNodelet : public nodelet::Nodelet
  {
    public:
      virtual ~DriverNodelet ();
    private:
      typedef FreenectConfig Config;
      typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
      typedef diagnostic_updater::FrequencyStatusParam FrequencyStatusParam;
      typedef diagnostic_updater::HeaderlessTopicDiagnostic TopicDiagnostic;
      typedef boost::shared_ptr<TopicDiagnostic> TopicDiagnosticPtr;

      /** \brief Nodelet initialization routine. */
      virtual void onInit ();
      void onInitImpl ();
      void setupDevice ();
      void updateModeMaps ();
      void startSynchronization ();
      void stopSynchronization ();

      /// @todo Consolidate all the mode stuff, maybe even in different class/header
      int mapMode2ConfigMode (const OutputMode& output_mode) const;
      OutputMode mapConfigMode2OutputMode (int mode) const;

      // Callback methods
      void rgbCb(const ImageBuffer& image, void* cookie);
      void depthCb(const ImageBuffer& depth_image, void* cookie);
      void irCb(const ImageBuffer&_image, void* cookie);
      void configCb(Config &config, uint32_t level);

      void rgbConnectCb();
      void depthConnectCb();
      void irConnectCb();

      // Methods to get calibration parameters for the various cameras
      sensor_msgs::CameraInfoPtr getDefaultCameraInfo(int width, int height, double f) const;
      sensor_msgs::CameraInfoPtr getRgbCameraInfo(const ImageBuffer& image, ros::Time time) const;
      sensor_msgs::CameraInfoPtr getIrCameraInfo(const ImageBuffer& image, ros::Time time) const;
      sensor_msgs::CameraInfoPtr getDepthCameraInfo(const ImageBuffer& image, ros::Time time) const;
      sensor_msgs::CameraInfoPtr getProjectorCameraInfo(const ImageBuffer& image, ros::Time time) const;

      // published topics
      image_transport::CameraPublisher pub_rgb_;
      image_transport::CameraPublisher pub_depth_, pub_depth_registered_;
      image_transport::CameraPublisher pub_ir_;
      ros::Publisher pub_projector_info_;

      // Maintain frequency diagnostics on all sensors
      boost::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
      double pub_freq_max_, pub_freq_min_;
      TopicDiagnosticPtr pub_rgb_freq_;
      bool enable_rgb_diagnostics_;
      TopicDiagnosticPtr pub_depth_freq_;
      bool enable_depth_diagnostics_;
      TopicDiagnosticPtr pub_ir_freq_;
      bool enable_ir_diagnostics_;
      boost::thread diagnostics_thread_;
      void updateDiagnostics();
      bool close_diagnostics_;

      bool close_tiltThread_;
      boost::thread tilt_thread_;
      TiltDriver tiltDriver_;

      bool motor_processing_;
      bool audio_processing_;
      bool rgb_processing_;
      bool ir_processing_;
      bool depth_processing_;
      freenect_device_flags subdevs_;
      // publish methods
      void publishRgbImage(const ImageBuffer& image, ros::Time time) const;
      void publishDepthImage(const ImageBuffer& depth, ros::Time time) const;
      void publishIrImage(const ImageBuffer& ir, ros::Time time) const;

      /** \brief the actual openni device */
      boost::shared_ptr<FreenectDevice> device_;
      boost::thread init_thread_;
      boost::mutex connect_mutex_;

      /** \brief reconfigure server*/
      boost::shared_ptr<ReconfigureServer> reconfigure_server_;
      Config config_;

      /** \brief Camera info manager objects. */
      boost::shared_ptr<camera_info_manager::CameraInfoManager> rgb_info_manager_, ir_info_manager_;
      std::string rgb_frame_id_;
      std::string depth_frame_id_;
      double depth_ir_offset_x_;
      double depth_ir_offset_y_;
      int z_offset_mm_;

      // Counters/flags for skipping frames
      boost::mutex counter_mutex_;
      int rgb_frame_counter_;
      int depth_frame_counter_;
      int ir_frame_counter_;
      bool publish_rgb_;
      bool publish_ir_;
      bool publish_depth_;
      void checkFrameCounters();

      void watchDog(const ros::TimerEvent& event);

      /** \brief timeout value in seconds to throw TIMEOUT exception */
      double time_out_;
      ros::Time rgb_time_stamp_;
      ros::Time depth_time_stamp_;
      ros::Time ir_time_stamp_;
      ros::Time time_stamp_;
      ros::Timer watch_dog_timer_;

      /** \brief enable libfreenect debugging */
      bool libfreenect_debug_;

      std::map<OutputMode, int> mode2config_map_;
      std::map<int, OutputMode> config2mode_map_;
      unsigned int num_frame_;
  };
}

#endif
