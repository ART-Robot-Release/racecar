/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011 Willow Garage, Inc.
 *    Suat Gedikli <gedikli@willowgarage.com>
 *    Patrick Michelich <michelich@willowgarage.com>
 *    Radu Bogdan Rusu <rusu@willowgarage.com>
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
#include "driver.h" /// @todo Get rid of this header entirely?
#include "openni_camera/openni_device_kinect.h"
#include "openni_camera/openni_image.h"
#include "openni_camera/openni_depth_image.h"
#include "openni_camera/openni_ir_image.h"
#include "openni_camera/openni_image_yuv_422.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <boost/algorithm/string/replace.hpp>

#include <log4cxx/logger.h>

using namespace std;
using namespace openni_wrapper;
namespace openni_camera
{
inline bool operator == (const XnMapOutputMode& mode1, const XnMapOutputMode& mode2)
{
  return (mode1.nXRes == mode2.nXRes && mode1.nYRes == mode2.nYRes && mode1.nFPS == mode2.nFPS);
}
inline bool operator != (const XnMapOutputMode& mode1, const XnMapOutputMode& mode2)
{
  return !(mode1 == mode2);
}

DriverNodelet::~DriverNodelet ()
{
  // If we're still stuck in initialization (e.g. can't connect to device), break out
  init_thread_.interrupt();
  init_thread_.join();

  // Join OpenNI wrapper threads, which call into rgbCb() etc. Those may use device_ methods,
  // so make sure they've finished before destroying device_.
  if (device_)
    device_->shutdown();

  /// @todo Test watchdog timer for race conditions. May need to use a separate callback queue
  /// controlled by the driver nodelet.
}

void DriverNodelet::onInit ()
{
  ros::NodeHandle& param_nh = getPrivateNodeHandle(); 

  config_init_ = false;

  // Initialize the sensor, but don't start any streams yet. That happens in the connection callbacks.
  updateModeMaps();
  setupDevice();

  // Initialize dynamic reconfigure
  reconfigure_server_.reset( new ReconfigureServer(param_nh) );
  reconfigure_server_->setCallback(boost::bind(&DriverNodelet::configCb, this, _1, _2));
  
  // Setting up device can take awhile but onInit shouldn't block, so we spawn a
  // new thread to do all the initialization
  init_thread_ = boost::thread(boost::bind(&DriverNodelet::onInitImpl, this));
}

void DriverNodelet::onInitImpl ()
{
  ros::NodeHandle& nh       = getNodeHandle();        // topics
  ros::NodeHandle& param_nh = getPrivateNodeHandle(); // parameters

  // Allow remapping namespaces rgb, ir, depth, depth_registered
  image_transport::ImageTransport it(nh);
  ros::NodeHandle rgb_nh(nh, "rgb");
  image_transport::ImageTransport rgb_it(rgb_nh);
  ros::NodeHandle ir_nh(nh, "ir");
  image_transport::ImageTransport ir_it(ir_nh);
  ros::NodeHandle depth_nh(nh, "depth");
  image_transport::ImageTransport depth_it(depth_nh);
  ros::NodeHandle depth_registered_nh(nh, "depth_registered");
  image_transport::ImageTransport depth_registered_it(depth_registered_nh);
  ros::NodeHandle projector_nh(nh, "projector");

  rgb_frame_counter_ = depth_frame_counter_ = ir_frame_counter_ = 0;
  publish_rgb_ = publish_ir_ = publish_depth_ = true;


  // Camera TF frames
  param_nh.param("rgb_frame_id",   rgb_frame_id_,   string("/openni_rgb_optical_frame"));
  param_nh.param("depth_frame_id", depth_frame_id_, string("/openni_depth_optical_frame"));
  NODELET_INFO("rgb_frame_id = '%s' ",   rgb_frame_id_.c_str());
  NODELET_INFO("depth_frame_id = '%s' ", depth_frame_id_.c_str());

  // Pixel offset between depth and IR images.
  // By default assume offset of (5,4) from 9x7 correlation window.
  // NOTE: These are now (temporarily?) dynamically reconfigurable, to allow tweaking.
  //param_nh.param("depth_ir_offset_x", depth_ir_offset_x_, 5.0);
  //param_nh.param("depth_ir_offset_y", depth_ir_offset_y_, 4.0);

  // The camera names are set to [rgb|depth]_[serial#], e.g. depth_B00367707227042B.
  // camera_info_manager substitutes this for ${NAME} in the URL.
  std::string serial_number = device_->getSerialNumber();
  std::string rgb_name, ir_name;
  if (serial_number.empty())
  {
    rgb_name = "rgb";
    ir_name  = "depth"; /// @todo Make it ir instead?
  }
  else
  {
    rgb_name = "rgb_"   + serial_number;
    ir_name  = "depth_" + serial_number;
  }

  std::string rgb_info_url, ir_info_url;
  param_nh.param("rgb_camera_info_url", rgb_info_url, std::string());
  param_nh.param("depth_camera_info_url", ir_info_url, std::string());

  // Suppress some of the output from loading camera calibrations (kinda hacky)
  log4cxx::LoggerPtr logger_ccp = log4cxx::Logger::getLogger("ros.camera_calibration_parsers");
  log4cxx::LoggerPtr logger_cim = log4cxx::Logger::getLogger("ros.camera_info_manager");
  logger_ccp->setLevel(log4cxx::Level::getFatal());
  logger_cim->setLevel(log4cxx::Level::getWarn());
  // Also suppress sync warnings from image_transport::CameraSubscriber. When subscribing to
  // depth_registered/foo with OpenNI registration disabled, the rectify nodelet for depth_registered/
  // will complain because it receives depth_registered/camera_info (from the register nodelet), but
  // the driver is not publishing depth_registered/image_raw.
  log4cxx::LoggerPtr logger_its = log4cxx::Logger::getLogger("ros.image_transport.sync");
  logger_its->setLevel(log4cxx::Level::getError());
  ros::console::notifyLoggerLevelsChanged();
  
  // Load the saved calibrations, if they exist
  rgb_info_manager_ = boost::make_shared<camera_info_manager::CameraInfoManager>(rgb_nh, rgb_name, rgb_info_url);
  ir_info_manager_  = boost::make_shared<camera_info_manager::CameraInfoManager>(ir_nh,  ir_name,  ir_info_url);

  if (!rgb_info_manager_->isCalibrated())
    NODELET_WARN("Using default parameters for RGB camera calibration.");
  if (!ir_info_manager_->isCalibrated())
    NODELET_WARN("Using default parameters for IR camera calibration.");

  // Advertise all published topics
  {
    // Prevent connection callbacks from executing until we've set all the publishers. Otherwise
    // connectCb() can fire while we're advertising (say) "depth/image_raw", but before we actually
    // assign to pub_depth_. Then pub_depth_.getNumSubscribers() returns 0, and we fail to start
    // the depth generator.
    boost::lock_guard<boost::mutex> lock(connect_mutex_);
    
    // Asus Xtion PRO does not have an RGB camera
    if (device_->hasImageStream())
    {
      image_transport::SubscriberStatusCallback itssc = boost::bind(&DriverNodelet::rgbConnectCb, this);
      ros::SubscriberStatusCallback rssc = boost::bind(&DriverNodelet::rgbConnectCb, this);
      pub_rgb_ = rgb_it.advertiseCamera("image_raw", 1, itssc, itssc, rssc, rssc);
    }

    if (device_->hasIRStream())
    {
      image_transport::SubscriberStatusCallback itssc = boost::bind(&DriverNodelet::irConnectCb, this);
      ros::SubscriberStatusCallback rssc = boost::bind(&DriverNodelet::irConnectCb, this);
      pub_ir_ = ir_it.advertiseCamera("image_raw", 1, itssc, itssc, rssc, rssc);
    }

    if (device_->hasDepthStream())
    {
      image_transport::SubscriberStatusCallback itssc = boost::bind(&DriverNodelet::depthConnectCb, this);
      ros::SubscriberStatusCallback rssc = boost::bind(&DriverNodelet::depthConnectCb, this);
      pub_depth_ = depth_it.advertiseCamera("image_raw", 1, itssc, itssc, rssc, rssc);
      pub_projector_info_ = projector_nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1, rssc, rssc);
      
      if (device_->isDepthRegistrationSupported())
        pub_depth_registered_ = depth_registered_it.advertiseCamera("image_raw", 1, itssc, itssc, rssc, rssc);
    }
  }

  // Create watch dog timer callback
  if (param_nh.getParam("time_out", time_out_) && time_out_ > 0.0)
  {
    time_stamp_ = ros::Time(0,0);
    watch_dog_timer_ = nh.createTimer(ros::Duration(time_out_), &DriverNodelet::watchDog, this);
  }
}

void DriverNodelet::setupDevice ()
{
  // Initialize the openni device
  OpenNIDriver& driver = OpenNIDriver::getInstance ();

  do {
    driver.updateDeviceList ();

    if (driver.getNumberDevices () == 0)
    {
      NODELET_INFO ("No devices connected.... waiting for devices to be connected");
      boost::this_thread::sleep(boost::posix_time::seconds(3));
      continue;
    }

    NODELET_INFO ("Number devices connected: %d", driver.getNumberDevices ());
    for (unsigned deviceIdx = 0; deviceIdx < driver.getNumberDevices (); ++deviceIdx)
    {
      NODELET_INFO("%u. device on bus %03u:%02u is a %s (%03x) from %s (%03x) with serial id \'%s\'",
                   deviceIdx + 1, driver.getBus(deviceIdx), driver.getAddress(deviceIdx),
                   driver.getProductName(deviceIdx), driver.getProductID(deviceIdx),
                   driver.getVendorName(deviceIdx), driver.getVendorID(deviceIdx),
                   driver.getSerialNumber(deviceIdx));
    }

    try {
      string device_id;
      int device_id_int;
      if (!getPrivateNodeHandle().getParam("device_id", device_id) &&
          !getPrivateNodeHandle().getParam("device_id", device_id_int))
      {
        
        NODELET_WARN ("~device_id is not set! Using first device.");
        device_ = driver.getDeviceByIndex (0);
      }
      else if (device_id.find ('@') != string::npos)
      {
        size_t pos = device_id.find ('@');
        unsigned bus = atoi (device_id.substr (0, pos).c_str ());
        unsigned address = atoi (device_id.substr (pos + 1, device_id.length () - pos - 1).c_str ());
        NODELET_INFO ("Searching for device with bus@address = %d@%d", bus, address);
        device_ = driver.getDeviceByAddress (bus, address);
      }
      else if (device_id[0] == '#')
      {
        unsigned index = atoi (device_id.c_str () + 1);
        NODELET_INFO ("Searching for device with index = %d", index);
        device_ = driver.getDeviceByIndex (index - 1);
      }
      else
      {
        if (device_id.empty()) // The ID passed contains only numbers
        {  
            std::stringstream ss;
            ss << device_id_int;
            device_id = ss.str();
        }
        NODELET_INFO ("Searching for device with serial number = '%s'", device_id.c_str ());
        device_ = driver.getDeviceBySerialNumber (device_id);
      }
    }
    catch (const OpenNIException& exception)
    {
      if (!device_)
      {
        NODELET_INFO ("No matching device found.... waiting for devices. Reason: %s", exception.what ());
        boost::this_thread::sleep(boost::posix_time::seconds(3));
        continue;
      }
      else
      {
        NODELET_ERROR ("Could not retrieve device. Reason: %s", exception.what ());
        exit (-1);
      }
    }
  } while (!device_);

  NODELET_INFO ("Opened '%s' on bus %d:%d with serial number '%s'", device_->getProductName (),
                device_->getBus (), device_->getAddress (), device_->getSerialNumber ());

  device_->registerImageCallback(&DriverNodelet::rgbCb,   *this);
  device_->registerDepthCallback(&DriverNodelet::depthCb, *this);
  device_->registerIRCallback   (&DriverNodelet::irCb,    *this);
}

void DriverNodelet::rgbConnectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  bool need_rgb = pub_rgb_.getNumSubscribers() > 0;
  
  if (need_rgb && !device_->isImageStreamRunning())
  {
    // Can't stream IR and RGB at the same time. Give RGB preference.
    if (device_->isIRStreamRunning())
    {
      NODELET_ERROR("Cannot stream RGB and IR at the same time. Streaming RGB only.");
      device_->stopIRStream();
    }
    
    device_->startImageStream();
    startSynchronization();
    time_stamp_ = ros::Time(0,0); // starting an additional stream blocks for a while, could upset watchdog
  }
  else if (!need_rgb && device_->isImageStreamRunning())
  {
    stopSynchronization();
    device_->stopImageStream();

    // Start IR if it's been blocked on RGB subscribers
    bool need_ir = pub_ir_.getNumSubscribers() > 0;
    if (need_ir && !device_->isIRStreamRunning())
    {
      device_->startIRStream();
      time_stamp_ = ros::Time(0,0);
    }
  }
}

void DriverNodelet::depthConnectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  /// @todo pub_projector_info_? Probably also subscribed to a depth image if you need it
  bool need_depth =
    device_->isDepthRegistered() ? pub_depth_registered_.getNumSubscribers() > 0 : pub_depth_.getNumSubscribers() > 0;
  /// @todo Warn if requested topics don't agree with OpenNI registration setting

  if (need_depth && !device_->isDepthStreamRunning())
  {
    device_->startDepthStream();
    startSynchronization();
    time_stamp_ = ros::Time(0,0); // starting an additional stream blocks for a while, could upset watchdog
  }
  else if (!need_depth && device_->isDepthStreamRunning())
  {
    stopSynchronization();
    device_->stopDepthStream();
  }
}

void DriverNodelet::irConnectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  bool need_ir = pub_ir_.getNumSubscribers() > 0;
  
  if (need_ir && !device_->isIRStreamRunning())
  {
    // Can't stream IR and RGB at the same time
    if (device_->isImageStreamRunning())
    {
      NODELET_ERROR("Cannot stream RGB and IR at the same time. Streaming RGB only.");
    }
    else
    {
      device_->startIRStream();
      time_stamp_ = ros::Time(0,0); // starting an additional stream blocks for a while, could upset watchdog
    }
  }
  else if (!need_ir)
  {
    device_->stopIRStream();
  }
}

// If any stream is ready, publish next available image from all streams
// This publishes all available data with a maximum time offset of one frame between any two sources
// Need to have lock to call this, since callbacks can be in different threads
void DriverNodelet::checkFrameCounters()
{
    if (max(rgb_frame_counter_, max(depth_frame_counter_, ir_frame_counter_)) > config_.data_skip) {
        // Reset all counters after we trigger publish
        rgb_frame_counter_   = 0;
        depth_frame_counter_ = 0;
        ir_frame_counter_    = 0;

        // Trigger publish on all topics
        publish_rgb_   = true;
        publish_depth_ = true;
        publish_ir_    = true;
    }
}

void DriverNodelet::rgbCb(boost::shared_ptr<openni_wrapper::Image> image, void* cookie)
{
  if (!config_init_)
    return;

  ros::Time time = ros::Time::now () + ros::Duration(config_.image_time_offset);
  time_stamp_ = time; // for watchdog

  bool publish = false;
  {
      boost::unique_lock<boost::mutex> counter_lock(counter_mutex_);
      rgb_frame_counter_++;
      checkFrameCounters();
      publish = publish_rgb_;

      if (publish)
          rgb_frame_counter_ = 0; // Reset counter if we publish this message to avoid under-throttling
  }

  if (publish)
      publishRgbImage(*image, time);

  publish_rgb_ = false;
}

void DriverNodelet::depthCb(boost::shared_ptr<openni_wrapper::DepthImage> depth_image, void* cookie)
{
  if (!config_init_)
    return;

  ros::Time time = ros::Time::now () + ros::Duration(config_.depth_time_offset);
  time_stamp_ = time; // for watchdog

  bool publish = false;
  {
      boost::unique_lock<boost::mutex> counter_lock(counter_mutex_);
      depth_frame_counter_++;
      checkFrameCounters();
      publish = publish_depth_;

      if (publish)
          depth_frame_counter_ = 0; // Reset counter if we publish this message to avoid under-throttling
  }

  if (publish)
      publishDepthImage(*depth_image, time);

  publish_depth_ = false;
}

void DriverNodelet::irCb(boost::shared_ptr<openni_wrapper::IRImage> ir_image, void* cookie)
{
  if (!config_init_)
    return;

  ros::Time time = ros::Time::now() + ros::Duration(config_.depth_time_offset);
  time_stamp_ = time; // for watchdog

  bool publish = false;
  {
      boost::unique_lock<boost::mutex> counter_lock(counter_mutex_);
      ir_frame_counter_++;
      checkFrameCounters();
      publish = publish_ir_;

      if (publish)
          ir_frame_counter_ = 0; // Reset counter if we publish this message to avoid under-throttling
  }

  if (publish)
      publishIrImage(*ir_image, time);
  publish_ir_ = false;
}

void DriverNodelet::publishRgbImage(const openni_wrapper::Image& image, ros::Time time) const
{
  sensor_msgs::ImagePtr rgb_msg = boost::make_shared<sensor_msgs::Image >();
  rgb_msg->header.stamp = time;
  rgb_msg->header.frame_id = rgb_frame_id_;
  bool downscale = false;
  if (image.getEncoding() == openni_wrapper::Image::BAYER_GRBG)
  {
    if (image.getWidth() == image_width_ && image.getHeight() == image_height_)
    {
      // image sizes match, we can copy directly
      rgb_msg->encoding = sensor_msgs::image_encodings::BAYER_GRBG8;
      rgb_msg->step = image_width_;
    }
    else
    {
      // image sizes missmatch, we have to downscale and de-bayer in this function
      rgb_msg->encoding = sensor_msgs::image_encodings::RGB8;
      rgb_msg->step = image_width_ * 3;
      downscale = true;
    }
  }
  else if (image.getEncoding() == openni_wrapper::Image::YUV422)
  {
    if (image.getWidth() == image_width_ && image.getHeight() == image_height_)
    {
      // image sizes match, we can copy directly
      rgb_msg->encoding = sensor_msgs::image_encodings::YUV422;
      rgb_msg->step = image_width_ * 2; // 4 bytes for 2 pixels
    }
    else
    {
      // image sizes missmatch, we have to downscale and convert in this function
      rgb_msg->encoding = sensor_msgs::image_encodings::RGB8;
      rgb_msg->step = image_width_ * 3;
      downscale = true;
    }
  }
  rgb_msg->height = image_height_;
  rgb_msg->width = image_width_;
  rgb_msg->data.resize(rgb_msg->height * rgb_msg->step);
  
  try
  {
    if (downscale)
    {
      if (image.getEncoding() == openni_wrapper::Image::BAYER_GRBG)
      {
        openni_wrapper::ImageBayerGRBG bayer_image(image.getMetaDataPtr(), openni_wrapper::ImageBayerGRBG::Bilinear);
        bayer_image.fillRGB(image_width_, image_height_, &rgb_msg->data[0]);
      }
      else if (image.getEncoding() == openni_wrapper::Image::YUV422)
      {
        openni_wrapper::ImageYUV422 yuv_image(image.getMetaDataPtr());
        yuv_image.fillRGB(image_width_, image_height_, &rgb_msg->data[0]);
      }
    }
    else
      image.fillRaw(&rgb_msg->data[0]);
  }
  catch ( OpenNIException e )
  {
    ROS_ERROR_THROTTLE(1,"%s",e.what());
  }
  
  pub_rgb_.publish(rgb_msg, getRgbCameraInfo(rgb_msg->width,rgb_msg->height,time));
}

void DriverNodelet::publishDepthImage(const openni_wrapper::DepthImage& depth, ros::Time time) const
{
  bool registered = device_->isDepthRegistered();
  
  /// @todo Get rid of depth_height_, depth_width_
  sensor_msgs::ImagePtr depth_msg = boost::make_shared<sensor_msgs::Image>();
  depth_msg->header.stamp    = time;
  depth_msg->encoding        = sensor_msgs::image_encodings::TYPE_16UC1;
  depth_msg->width           = depth_width_;
  depth_msg->height          = depth_height_;
  depth_msg->step            = depth_msg->width * sizeof(short);
  depth_msg->data.resize(depth_msg->height * depth_msg->step);

  try
  {
    depth.fillDepthImageRaw(depth_msg->width, depth_msg->height, reinterpret_cast<unsigned short*>(&depth_msg->data[0]),
                            depth_msg->step);
  }
  catch ( OpenNIException e )
  {
    ROS_ERROR_THROTTLE(1,"%s",e.what());
  }

  if (fabs(z_scaling_ - 1.0) > 1e-6)
  {
    uint16_t* data = reinterpret_cast<uint16_t*>(&depth_msg->data[0]);
    for (unsigned int i = 0; i < depth_msg->width * depth_msg->height; ++i)
      if (data[i] != 0)
	    data[i] = static_cast<uint16_t>(data[i] * z_scaling_);
  }

  if (z_offset_mm_ != 0)
  {
    uint16_t* data = reinterpret_cast<uint16_t*>(&depth_msg->data[0]);
    for (unsigned int i = 0; i < depth_msg->width * depth_msg->height; ++i)
      if (data[i] != 0)
	    data[i] += z_offset_mm_;
  }

  if (registered)
  {
    // Publish RGB camera info and raw depth image to depth_registered/ ns
    depth_msg->header.frame_id = rgb_frame_id_;
    pub_depth_registered_.publish(depth_msg, getRgbCameraInfo(depth_msg->width, depth_msg->height, time));
  }
  else
  {
    // Publish depth camera info and raw depth image to depth/ ns
    depth_msg->header.frame_id = depth_frame_id_;
    pub_depth_.publish(depth_msg, getDepthCameraInfo(depth_msg->width, depth_msg->height, time));
  }

  // Projector "info" probably only useful for working with disparity images
  if (pub_projector_info_.getNumSubscribers() > 0)
  {
    pub_projector_info_.publish(getProjectorCameraInfo(depth_msg->width, depth_msg->height, time));
  }
}

void DriverNodelet::publishIrImage(const openni_wrapper::IRImage& ir, ros::Time time) const
{
  sensor_msgs::ImagePtr ir_msg = boost::make_shared<sensor_msgs::Image>();
  ir_msg->header.stamp    = time;
  ir_msg->header.frame_id = depth_frame_id_;
  ir_msg->encoding        = sensor_msgs::image_encodings::MONO16;
  ir_msg->height          = ir.getHeight();
  ir_msg->width           = ir.getWidth();
  ir_msg->step            = ir_msg->width * sizeof(uint16_t);
  ir_msg->data.resize(ir_msg->height * ir_msg->step);

  try
  {
    ir.fillRaw(ir.getWidth(), ir.getHeight(), reinterpret_cast<unsigned short*>(&ir_msg->data[0]));
  }
  catch ( OpenNIException e )
  {
    ROS_ERROR_THROTTLE(1,"%s",e.what());
  }

  pub_ir_.publish(ir_msg, getIrCameraInfo(ir.getWidth(), ir.getHeight(), time));
}

sensor_msgs::CameraInfoPtr DriverNodelet::getDefaultCameraInfo(int width, int height, double f) const
{
  sensor_msgs::CameraInfoPtr info = boost::make_shared<sensor_msgs::CameraInfo>();

  info->width  = width;
  info->height = height;

  // No distortion
  info->D.resize(5, 0.0);
  info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

  // Simple camera matrix: square pixels (fx = fy), principal point at center
  info->K.assign(0.0);
  info->K[0] = info->K[4] = f;
  info->K[2] = (width / 2) - 0.5;
  // Aspect ratio for the camera center on Kinect (and other devices?) is 4/3
  // This formula keeps the principal point the same in VGA and SXGA modes
  info->K[5] = (width * (3./8.)) - 0.5;
  info->K[8] = 1.0;

  // No separate rectified image plane, so R = I
  info->R.assign(0.0);
  info->R[0] = info->R[4] = info->R[8] = 1.0;

  // Then P=K(I|0) = (K|0)
  info->P.assign(0.0);
  info->P[0]  = info->P[5] = f; // fx, fy
  info->P[2]  = info->K[2];     // cx
  info->P[6]  = info->K[5];     // cy
  info->P[10] = 1.0;

  return info;
}

/// @todo Use binning/ROI properly in publishing camera infos
sensor_msgs::CameraInfoPtr DriverNodelet::getRgbCameraInfo(int width, int height, ros::Time time) const
{
  sensor_msgs::CameraInfoPtr info;

  if (rgb_info_manager_->isCalibrated())
  {
    info = boost::make_shared<sensor_msgs::CameraInfo>(rgb_info_manager_->getCameraInfo());
    if ( info->width != width )
    {
      // Use uncalibrated values
      ROS_WARN_ONCE("Image resolution doesn't match calibration of the RGB camera. Using default parameters.");
      info = getDefaultCameraInfo(width, height, device_->getImageFocalLength(width));
    }
  }
  else
  {
    // If uncalibrated, fill in default values
    info = getDefaultCameraInfo(width, height, device_->getImageFocalLength(width));
  }

  // Fill in header
  info->header.stamp    = time;
  info->header.frame_id = rgb_frame_id_;

  return info;
}

sensor_msgs::CameraInfoPtr DriverNodelet::getIrCameraInfo(int width, int height, ros::Time time) const
{
  sensor_msgs::CameraInfoPtr info;

  if (ir_info_manager_->isCalibrated())
  {
    info = boost::make_shared<sensor_msgs::CameraInfo>(ir_info_manager_->getCameraInfo());
    if ( info->width != width )
    {
      // Use uncalibrated values
      ROS_WARN_ONCE("Image resolution doesn't match calibration of the IR camera. Using default parameters.");
      info = getDefaultCameraInfo(width, height, device_->getImageFocalLength(width));
    }
  }
  else
  {
    // If uncalibrated, fill in default values
    info = getDefaultCameraInfo(width, height, device_->getDepthFocalLength(width));
  }

  // Fill in header
  info->header.stamp    = time;
  info->header.frame_id = depth_frame_id_;

  return info;
}

sensor_msgs::CameraInfoPtr DriverNodelet::getDepthCameraInfo(int width, int height, ros::Time time) const
{
  // The depth image has essentially the same intrinsics as the IR image, BUT the
  // principal point is offset by half the size of the hardware correlation window
  // (probably 9x9 or 9x7 in 640x480 mode). See http://www.ros.org/wiki/kinect_calibration/technical

  double scaling = (double)width / 640;

  sensor_msgs::CameraInfoPtr info = getIrCameraInfo(width, height, time);
  info->K[2] -= depth_ir_offset_x_*scaling; // cx
  info->K[5] -= depth_ir_offset_y_*scaling; // cy
  info->P[2] -= depth_ir_offset_x_*scaling; // cx
  info->P[6] -= depth_ir_offset_y_*scaling; // cy

  /// @todo Could put this in projector frame so as to encode the baseline in P[3]
  return info;
}

sensor_msgs::CameraInfoPtr DriverNodelet::getProjectorCameraInfo(int width, int height, ros::Time time) const
{
  // The projector info is simply the depth info with the baseline encoded in the P matrix.
  // It's only purpose is to be the "right" camera info to the depth camera's "left" for
  // processing disparity images.
  sensor_msgs::CameraInfoPtr info = getDepthCameraInfo(width, height, time);
  // Tx = -baseline * fx
  info->P[3] = -device_->getBaseline() * info->P[0];
  return info;
}

void DriverNodelet::configCb(Config &config, uint32_t level)
{
  depth_ir_offset_x_ = config.depth_ir_offset_x;
  depth_ir_offset_y_ = config.depth_ir_offset_y;
  z_offset_mm_ = config.z_offset_mm;
  z_scaling_ = config.z_scaling;

  // Based on the config options, try to set the depth/image mode
  // The device driver might decide to switch to a 'compatible mode',
  // i.e. a higher resolution than the requested one, in which case it
  // will later downsample the image.
  // Only if the 'compatible mode' is different from the current one, we will
  // need to reset the streams.

  bool depth_mode_changed = false;
  bool image_mode_changed = false;

  XnMapOutputMode compatible_image_mode;
  XnMapOutputMode compatible_depth_mode;

  if (device_->hasImageStream ())
  {
    XnMapOutputMode image_mode = mapConfigMode2XnMode (config.image_mode);
    image_width_  = image_mode.nXRes;
    image_height_ = image_mode.nYRes;

    // does the device support the new image mode?
    if (!device_->findCompatibleImageMode (image_mode, compatible_image_mode))
    {
      XnMapOutputMode default_mode = device_->getDefaultImageMode();
      NODELET_WARN("Could not find any compatible image output mode for %d x %d @ %d. "
                   "Falling back to default image output mode %d x %d @ %d.",
                    image_mode.nXRes, image_mode.nYRes, image_mode.nFPS,
                    default_mode.nXRes, default_mode.nYRes, default_mode.nFPS);

      config.image_mode = mapXnMode2ConfigMode(default_mode);
      image_mode = compatible_image_mode = default_mode;
    }

    if ( compatible_image_mode != device_->getImageOutputMode () )
    {
      image_mode_changed = true;
    }
  }
  
  if (device_->hasDepthStream())
  {
    XnMapOutputMode depth_mode = mapConfigMode2XnMode (config.depth_mode);
    depth_width_  = depth_mode.nXRes;
    depth_height_ = depth_mode.nYRes;

    // does the device support the new depth mode?
    if (!device_->findCompatibleDepthMode (depth_mode, compatible_depth_mode))
    {
      XnMapOutputMode default_mode = device_->getDefaultDepthMode();
      NODELET_WARN ("Could not find any compatible depth output mode for %d x %d @ %d. "
                    "Falling back to default depth output mode %d x %d @ %d.",
                    depth_mode.nXRes, depth_mode.nYRes, depth_mode.nFPS,
                    default_mode.nXRes, default_mode.nYRes, default_mode.nFPS);

      config.depth_mode = mapXnMode2ConfigMode(default_mode);
      depth_mode = compatible_depth_mode = default_mode;
    }

    if ( compatible_depth_mode != device_->getDepthOutputMode () )
    {
      depth_mode_changed = true;
    }
  }

  // here everything is fine. Now make the changes
  if ( image_mode_changed || depth_mode_changed )
  {
    // streams need to be reset!
    stopSynchronization();

    if ( image_mode_changed )
    {
      device_->setImageOutputMode (compatible_image_mode);
    }

    if ( depth_mode_changed )
    {
      device_->setDepthOutputMode (compatible_depth_mode);
    }

    startSynchronization ();
  }

  /// @todo Run connectCb if registration setting changes
  if (device_->isDepthRegistered () && !config.depth_registration)
  {
    device_->setDepthRegistration (false);
  }
  else if (!device_->isDepthRegistered () && config.depth_registration)
  {
    device_->setDepthRegistration (true);
  }

  // now we can copy
  config_ = config;

  config_init_ = true;
}

void DriverNodelet::startSynchronization()
{
  if (device_->isSynchronizationSupported() &&
      !device_->isSynchronized() &&
      device_->getImageOutputMode().nFPS == device_->getDepthOutputMode().nFPS &&
      device_->isImageStreamRunning() &&
      device_->isDepthStreamRunning() )
  {
    device_->setSynchronization(true);
  }
}

void DriverNodelet::stopSynchronization()
{
  if (device_->isSynchronizationSupported() &&
      device_->isSynchronized())
  {
    device_->setSynchronization(false);
  }
}

void DriverNodelet::updateModeMaps ()
{
  XnMapOutputMode output_mode;

  output_mode.nXRes = XN_SXGA_X_RES;
  output_mode.nYRes = XN_SXGA_Y_RES;
  output_mode.nFPS  = 15;
  xn2config_map_[output_mode] = OpenNI_SXGA_15Hz;
  config2xn_map_[OpenNI_SXGA_15Hz] = output_mode;

  output_mode.nXRes = XN_VGA_X_RES;
  output_mode.nYRes = XN_VGA_Y_RES;
  output_mode.nFPS  = 25;
  xn2config_map_[output_mode] = OpenNI_VGA_25Hz;
  config2xn_map_[OpenNI_VGA_25Hz] = output_mode;
  output_mode.nFPS  = 30;
  xn2config_map_[output_mode] = OpenNI_VGA_30Hz;
  config2xn_map_[OpenNI_VGA_30Hz] = output_mode;

  output_mode.nXRes = XN_QVGA_X_RES;
  output_mode.nYRes = XN_QVGA_Y_RES;
  output_mode.nFPS  = 25;
  xn2config_map_[output_mode] = OpenNI_QVGA_25Hz;
  config2xn_map_[OpenNI_QVGA_25Hz] = output_mode;
  output_mode.nFPS  = 30;
  xn2config_map_[output_mode] = OpenNI_QVGA_30Hz;
  config2xn_map_[OpenNI_QVGA_30Hz] = output_mode;
  output_mode.nFPS  = 60;
  xn2config_map_[output_mode] = OpenNI_QVGA_60Hz;
  config2xn_map_[OpenNI_QVGA_60Hz] = output_mode;

  output_mode.nXRes = XN_QQVGA_X_RES;
  output_mode.nYRes = XN_QQVGA_Y_RES;
  output_mode.nFPS  = 25;
  xn2config_map_[output_mode] = OpenNI_QQVGA_25Hz;
  config2xn_map_[OpenNI_QQVGA_25Hz] = output_mode;
  output_mode.nFPS  = 30;
  xn2config_map_[output_mode] = OpenNI_QQVGA_30Hz;
  config2xn_map_[OpenNI_QQVGA_30Hz] = output_mode;
  output_mode.nFPS  = 60;
  xn2config_map_[output_mode] = OpenNI_QQVGA_60Hz;
  config2xn_map_[OpenNI_QQVGA_60Hz] = output_mode;
}

int DriverNodelet::mapXnMode2ConfigMode (const XnMapOutputMode& output_mode) const
{
  std::map<XnMapOutputMode, int, modeComp>::const_iterator it = xn2config_map_.find (output_mode);

  if (it == xn2config_map_.end ())
  {
    NODELET_ERROR ("mode %dx%d@%d could not be found", output_mode.nXRes, output_mode.nYRes, output_mode.nFPS);
    exit (-1);
  }
  else
    return it->second;
}

XnMapOutputMode DriverNodelet::mapConfigMode2XnMode (int mode) const
{
  std::map<int, XnMapOutputMode>::const_iterator it = config2xn_map_.find (mode);
  if (it == config2xn_map_.end ())
  {
    NODELET_ERROR ("mode %d could not be found", mode);
    exit (-1);
  }
  else
    return it->second;
}

void DriverNodelet::watchDog (const ros::TimerEvent& event)
{
  /// @todo Also watch IR
  if ( !time_stamp_.isZero() && (device_->isDepthStreamRunning() || device_->isImageStreamRunning()) )
  {
    ros::Duration duration = ros::Time::now() - time_stamp_;
    if (duration.toSec() >= time_out_)
    {
      NODELET_ERROR("Timeout");
      watch_dog_timer_.stop();
      throw std::runtime_error("Timeout occured in DriverNodelet");
    }
  }
}

}

// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(openni_camera::DriverNodelet, nodelet::Nodelet)
