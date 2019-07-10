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
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <boost/algorithm/string/replace.hpp>
#include <log4cxx/logger.h>


using namespace std;
namespace freenect_camera {

DriverNodelet::~DriverNodelet ()
{
  // If we're still stuck in initialization (e.g. can't connect to device), break out
  init_thread_.interrupt();
  init_thread_.join();

  // Interrupt and close diagnostics thread
  close_diagnostics_ = true;
  diagnostics_thread_.join();
  if(motor_processing_)
  {
    close_tiltThread_ =true;
    tilt_thread_.join();
  }
  FreenectDriver& driver = FreenectDriver::getInstance(subdevs_);
  driver.shutdown();

  /// @todo Test watchdog timer for race conditions. May need to use a separate callback queue
  /// controlled by the driver nodelet.
}

void DriverNodelet::onInit ()
{

  // Setting up device can take awhile but onInit shouldn't block, so we spawn a
  // new thread to do all the initialization
  init_thread_ = boost::thread(boost::bind(&DriverNodelet::onInitImpl, this));
}

void DriverNodelet::onInitImpl ()
{

  ros::NodeHandle& nh       = this->getNodeHandle();        // topics
  ros::NodeHandle& param_nh = this->getPrivateNodeHandle(); // parameters
  num_frame_=1;
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

  // Check to see if we should enable debugging messages in libfreenect
  // libfreenect_debug_ should be set before calling setupDevice
  param_nh.param("debug" , libfreenect_debug_, false);

  // Initialize the sensor, but don't start any streams yet. That happens in the connection callbacks.
  updateModeMaps();

  //get subdevs
  param_nh.param("motor_processing", motor_processing_, false);
  param_nh.param("audio_processing", audio_processing_, true);
  param_nh.param("rgb_processing", rgb_processing_, false);
  param_nh.param("ir_processing", ir_processing_, false);
  param_nh.param("depth_processing", depth_processing_, false);
  subdevs_=(freenect_device_flags)(0);
  if(!(motor_processing_||audio_processing_||rgb_processing_||ir_processing_||depth_processing_))
  {
    subdevs_=(freenect_device_flags)(FREENECT_DEVICE_AUDIO);
  }
  if(motor_processing_) subdevs_=(freenect_device_flags)(subdevs_|FREENECT_DEVICE_MOTOR);
  if(audio_processing_)
  {
    subdevs_=(freenect_device_flags)(subdevs_|FREENECT_DEVICE_AUDIO);
  }
  if(rgb_processing_||ir_processing_||depth_processing_) subdevs_=(freenect_device_flags)(subdevs_|FREENECT_DEVICE_CAMERA);


  setupDevice();

  // Initialize dynamic reconfigure
  reconfigure_server_.reset( new ReconfigureServer(param_nh) );

  reconfigure_server_->setCallback(boost::bind(&DriverNodelet::configCb, this, _1, _2));
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

  double diagnostics_max_frequency, diagnostics_min_frequency;
  double diagnostics_tolerance, diagnostics_window_time;


  param_nh.param("enable_rgb_diagnostics", enable_rgb_diagnostics_, false);


  param_nh.param("enable_ir_diagnostics", enable_ir_diagnostics_, false);


  param_nh.param("enable_depth_diagnostics", enable_depth_diagnostics_, false);


  param_nh.param("diagnostics_max_frequency", diagnostics_max_frequency, 30.0);


  param_nh.param("diagnostics_min_frequency", diagnostics_min_frequency, 30.0);


  param_nh.param("diagnostics_tolerance", diagnostics_tolerance, 0.05);


  param_nh.param("diagnostics_window_time", diagnostics_window_time, 5.0);

  // Suppress some of the output from loading camera calibrations (kinda hacky)
  log4cxx::LoggerPtr logger_ccp = log4cxx::Logger::getLogger("ros.camera_calibration_parsers");
  log4cxx::LoggerPtr logger_cim = log4cxx::Logger::getLogger("ros.camera_info_manager");
  logger_ccp->setLevel(log4cxx::Level::getFatal());
  logger_cim->setLevel(log4cxx::Level::getWarn());
  // Also suppress sync warnings from image_transport::CameraSubscriber. When subscribing to
  // depth_registered/foo with Freenect registration disabled, the rectify nodelet for depth_registered/
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

    // Instantiate the diagnostic updater
    pub_freq_max_ = diagnostics_max_frequency;
    pub_freq_min_ = diagnostics_min_frequency;
    diagnostic_updater_.reset(new diagnostic_updater::Updater);

    // Set hardware id
    std::string hardware_id = std::string(device_->getProductName()) + "-" +
        std::string(device_->getSerialNumber());
    diagnostic_updater_->setHardwareID(hardware_id);


    // Asus Xtion PRO does not have an RGB camera
    if (device_->hasImageStream()&&rgb_processing_)
    {
      image_transport::SubscriberStatusCallback itssc = boost::bind(&DriverNodelet::rgbConnectCb, this);
      ros::SubscriberStatusCallback rssc = boost::bind(&DriverNodelet::rgbConnectCb, this);
      pub_rgb_ = rgb_it.advertiseCamera("image_raw", 1, itssc, itssc, rssc, rssc);
      if (enable_rgb_diagnostics_) {
        pub_rgb_freq_.reset(new TopicDiagnostic("RGB Image", *diagnostic_updater_,
            FrequencyStatusParam(&pub_freq_min_, &pub_freq_max_,
                diagnostics_tolerance, diagnostics_window_time)));
      }
    }

    if (device_->hasIRStream()&&ir_processing_)
    {
      image_transport::SubscriberStatusCallback itssc = boost::bind(&DriverNodelet::irConnectCb, this);
      ros::SubscriberStatusCallback rssc = boost::bind(&DriverNodelet::irConnectCb, this);
      pub_ir_ = ir_it.advertiseCamera("image_raw", 1, itssc, itssc, rssc, rssc);
      if (enable_ir_diagnostics_) {
        pub_ir_freq_.reset(new TopicDiagnostic("IR Image", *diagnostic_updater_,
            FrequencyStatusParam(&pub_freq_min_, &pub_freq_max_,
                diagnostics_tolerance, diagnostics_window_time)));
      }
    }

    if (device_->hasDepthStream()&&depth_processing_)
    {
      image_transport::SubscriberStatusCallback itssc = boost::bind(&DriverNodelet::depthConnectCb, this);
      ros::SubscriberStatusCallback rssc = boost::bind(&DriverNodelet::depthConnectCb, this);
      pub_depth_ = depth_it.advertiseCamera("image_raw", 1, itssc, itssc, rssc, rssc);
      if (enable_depth_diagnostics_) {
        pub_depth_freq_.reset(new TopicDiagnostic("Depth Image", *diagnostic_updater_,
            FrequencyStatusParam(&pub_freq_min_, &pub_freq_max_,
                diagnostics_tolerance, diagnostics_window_time)));
      }

      pub_projector_info_ = projector_nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1, rssc, rssc);

      if (device_->isDepthRegistrationSupported()) {
        pub_depth_registered_ = depth_registered_it.advertiseCamera("image_raw", 1, itssc, itssc, rssc, rssc);
      }
    }
  }

  //stop camera stream
  if(subdevs_&FREENECT_DEVICE_CAMERA)
  {
    stopSynchronization();
    if(!depth_processing_ && device_->isDepthStreamRunning())
    {
      device_->stopDepthStream();
    }
    if(!rgb_processing_ && device_->isImageStreamRunning())
    {
      device_->stopImageStream();
    }
    if(!ir_processing_ && device_->isIRStreamRunning())
    {
      device_->stopIRStream();
    }
    startSynchronization();
  }
  // Create separate diagnostics thread
  close_diagnostics_ = false;
  diagnostics_thread_ = boost::thread(boost::bind(&DriverNodelet::updateDiagnostics, this));

  // Create watch dog timer callback
  param_nh.param<double>("time_out", time_out_, 5.0);
  if (time_out_ > 0.0)
  {
    watch_dog_timer_ = nh.createTimer(ros::Duration(time_out_), &DriverNodelet::watchDog, this);
  }

  device_->publishersAreReady();

  // Create separate tilt motor thread
  if(motor_processing_)
  {
    close_tiltThread_ = false;
    tiltDriver_= TiltDriver(device_,&close_tiltThread_);
    tilt_thread_ = boost::thread(boost::bind(&TiltDriver::run, &tiltDriver_));
  }


}

void DriverNodelet::updateDiagnostics() {
  while (!close_diagnostics_) {
    diagnostic_updater_->update();
    boost::this_thread::sleep(boost::posix_time::milliseconds(10));
  }
}

void DriverNodelet::setupDevice ()
{
  // Initialize the openni device
  FreenectDriver& driver = FreenectDriver::getInstance(subdevs_);
  ROS_INFO("subdevs: %d\n",subdevs_);
  // Enable debugging in libfreenect if requested
  if (libfreenect_debug_)
    driver.enableDebug();

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
      if (!getPrivateNodeHandle().getParam("device_id", device_id))
      {
        NODELET_WARN ("~device_id is not set! Using first device.");
        device_ = driver.getDeviceByIndex(0);
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
        unsigned index = atoi (device_id.c_str() + 1);
        NODELET_INFO ("Searching for device with index = %d", index);
        device_ = driver.getDeviceByIndex (index - 1);
      }
      else
      {
        NODELET_INFO ("Searching for device with serial number = '%s'", device_id.c_str ());
        device_ = driver.getDeviceBySerialNumber (device_id);
      }
    }
    catch (exception& e)
    {
      if (!device_)
      {
        NODELET_INFO ("No matching device found.... waiting for devices. Reason: %s", e.what ());
        boost::this_thread::sleep(boost::posix_time::seconds(3));
        continue;
      }
      else
      {
        NODELET_FATAL ("Could not retrieve device. Reason: %s", e.what ());
        exit (-1);
      }
    }
  } while (!device_);

  NODELET_INFO ("Opened '%s' on bus %d:%d with serial number '%s'", device_->getProductName (),
                device_->getBus (), device_->getAddress (), device_->getSerialNumber ());

  if(subdevs_&FREENECT_DEVICE_CAMERA) device_->registerImageCallback(&DriverNodelet::rgbCb,   *this);
  if(subdevs_&FREENECT_DEVICE_CAMERA) device_->registerDepthCallback(&DriverNodelet::depthCb, *this);
  if(subdevs_&FREENECT_DEVICE_CAMERA) device_->registerIRCallback   (&DriverNodelet::irCb,    *this);


}

void DriverNodelet::rgbConnectCb()
{
  //std::cout << "rgb connect cb called";
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  //std::cout << "..." << std::endl;
  bool need_rgb = pub_rgb_.getNumSubscribers() > 0;
  //std::cout << "  need_rgb: " << need_rgb << std::endl;

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
    rgb_time_stamp_ = ros::Time::now(); // update stamp for watchdog
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
      ir_time_stamp_ = ros::Time::now(); // update stamp for watchdog
    }
  }
  //std::cout << "rgb connect cb end..." << std::endl;
}

void DriverNodelet::depthConnectCb()
{
  //std::cout << "depth connect cb called";
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  //std::cout << "..." << std::endl;
  /// @todo pub_projector_info_? Probably also subscribed to a depth image if you need it
  bool need_depth =
    device_->isDepthRegistered() ? pub_depth_registered_.getNumSubscribers() > 0 : pub_depth_.getNumSubscribers() > 0;
  /// @todo Warn if requested topics don't agree with Freenect registration setting
  //std::cout << "  need_depth: " << need_depth << std::endl;

  if (need_depth && !device_->isDepthStreamRunning())
  {
    device_->startDepthStream();
    startSynchronization();
    depth_time_stamp_ = ros::Time::now(); // update stamp for watchdog

  }
  else if (!need_depth && device_->isDepthStreamRunning())
  {
    stopSynchronization();
    device_->stopDepthStream();
  }
  //std::cout << "depth connect cb end..." << std::endl;
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
      ir_time_stamp_ = ros::Time::now(); // update stamp for watchdog
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

void DriverNodelet::rgbCb(const ImageBuffer& image, void* cookie)
{
  ros::Time time = ros::Time::now () + ros::Duration(config_.image_time_offset);
  rgb_time_stamp_ = time; // for watchdog

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
      publishRgbImage(image, time);

  publish_rgb_ = false;
}

void DriverNodelet::depthCb(const ImageBuffer& depth_image, void* cookie)
{
  ros::Time time = ros::Time::now () + ros::Duration(config_.depth_time_offset);
  depth_time_stamp_ = time; // for watchdog
  if((num_frame_%3==0))
  {
    num_frame_++;
    return;//20hz
  }
  num_frame_++;
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
      publishDepthImage(depth_image, time);

  publish_depth_ = false;
}

void DriverNodelet::irCb(const ImageBuffer& ir_image, void* cookie)
{
  ros::Time time = ros::Time::now() + ros::Duration(config_.depth_time_offset);
  ir_time_stamp_ = time; // for watchdog

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
      publishIrImage(ir_image, time);
  publish_ir_ = false;
}

void DriverNodelet::publishRgbImage(const ImageBuffer& image, ros::Time time) const
{
  //NODELET_INFO_THROTTLE(1.0, "rgb image callback called");
  sensor_msgs::ImagePtr rgb_msg = boost::make_shared<sensor_msgs::Image >();
  rgb_msg->header.stamp = time;
  rgb_msg->header.frame_id = rgb_frame_id_;
  rgb_msg->height = image.metadata.height;
  rgb_msg->width = image.metadata.width;
  switch(image.metadata.video_format) {
    case FREENECT_VIDEO_RGB:
      rgb_msg->encoding = sensor_msgs::image_encodings::RGB8;
      rgb_msg->step = rgb_msg->width * 3;
      break;
    case FREENECT_VIDEO_BAYER:
      rgb_msg->encoding = sensor_msgs::image_encodings::BAYER_GRBG8;
      rgb_msg->step = rgb_msg->width;
      break;
    case FREENECT_VIDEO_YUV_RGB:
      rgb_msg->encoding = sensor_msgs::image_encodings::YUV422;
      rgb_msg->step = rgb_msg->width * 2;
      break;
    default:
      NODELET_ERROR("Unknown RGB image format received from libfreenect");
      // Unknown encoding -- don't publish
      return;
  }
  rgb_msg->data.resize(rgb_msg->height * rgb_msg->step);
  fillImage(image, reinterpret_cast<void*>(&rgb_msg->data[0]));

  pub_rgb_.publish(rgb_msg, getRgbCameraInfo(image, time));
  if (enable_rgb_diagnostics_)
      pub_rgb_freq_->tick();
}

void DriverNodelet::publishDepthImage(const ImageBuffer& depth, ros::Time time) const
{
  //NODELET_INFO_THROTTLE(1.0, "depth image callback called");
  bool registered = depth.is_registered;

  sensor_msgs::ImagePtr depth_msg = boost::make_shared<sensor_msgs::Image>();
  depth_msg->header.stamp    = time;
  depth_msg->encoding        = sensor_msgs::image_encodings::TYPE_16UC1;
  depth_msg->height          = depth.metadata.height;
  depth_msg->width           = depth.metadata.width;
  depth_msg->step            = depth_msg->width * sizeof(short);
  depth_msg->data.resize(depth_msg->height * depth_msg->step);

  fillImage(depth, reinterpret_cast<void*>(&depth_msg->data[0]));

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
    pub_depth_registered_.publish(depth_msg, getRgbCameraInfo(depth, time));
  }
  else
  {
    // Publish depth camera info and raw depth image to depth/ ns
    depth_msg->header.frame_id = depth_frame_id_;
    pub_depth_.publish(depth_msg, getDepthCameraInfo(depth, time));
  }
  if (enable_depth_diagnostics_)
      pub_depth_freq_->tick();

  // Projector "info" probably only useful for working with disparity images
  if (pub_projector_info_.getNumSubscribers() > 0)
  {
    pub_projector_info_.publish(getProjectorCameraInfo(depth, time));
  }
}

void DriverNodelet::publishIrImage(const ImageBuffer& ir, ros::Time time) const
{
  sensor_msgs::ImagePtr ir_msg = boost::make_shared<sensor_msgs::Image>();
  ir_msg->header.stamp    = time;
  ir_msg->header.frame_id = depth_frame_id_;
  ir_msg->encoding        = sensor_msgs::image_encodings::MONO16;
  ir_msg->height          = ir.metadata.height;
  ir_msg->width           = ir.metadata.width;
  ir_msg->step            = ir_msg->width * sizeof(uint16_t);
  ir_msg->data.resize(ir_msg->height * ir_msg->step);

  fillImage(ir, reinterpret_cast<void*>(&ir_msg->data[0]));

  pub_ir_.publish(ir_msg, getIrCameraInfo(ir, time));

  if (enable_ir_diagnostics_)
      pub_ir_freq_->tick();
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
sensor_msgs::CameraInfoPtr DriverNodelet::getRgbCameraInfo(const ImageBuffer& image, ros::Time time) const
{
  sensor_msgs::CameraInfoPtr info;

  if (rgb_info_manager_->isCalibrated())
  {
    info = boost::make_shared<sensor_msgs::CameraInfo>(rgb_info_manager_->getCameraInfo());
  }
  else
  {
    // If uncalibrated, fill in default values
    info = getDefaultCameraInfo(image.metadata.width, image.metadata.height, image.focal_length);
  }

  // Fill in header
  info->header.stamp    = time;
  info->header.frame_id = rgb_frame_id_;

  return info;
}

sensor_msgs::CameraInfoPtr DriverNodelet::getIrCameraInfo(
    const ImageBuffer& image, ros::Time time) const {
  sensor_msgs::CameraInfoPtr info;

  if (ir_info_manager_->isCalibrated())
  {
    info = boost::make_shared<sensor_msgs::CameraInfo>(ir_info_manager_->getCameraInfo());
  }
  else
  {
    // If uncalibrated, fill in default values
    info = getDefaultCameraInfo(image.metadata.width, image.metadata.height, image.focal_length);
  }

  // Fill in header
  info->header.stamp    = time;
  info->header.frame_id = depth_frame_id_;

  return info;
}

sensor_msgs::CameraInfoPtr DriverNodelet::getDepthCameraInfo(
    const ImageBuffer& image, ros::Time time) const {
  // The depth image has essentially the same intrinsics as the IR image, BUT the
  // principal point is offset by half the size of the hardware correlation window
  // (probably 9x9 or 9x7). See http://www.ros.org/wiki/kinect_calibration/technical

  sensor_msgs::CameraInfoPtr info = getIrCameraInfo(image, time);
  info->K[2] -= depth_ir_offset_x_; // cx
  info->K[5] -= depth_ir_offset_y_; // cy
  info->P[2] -= depth_ir_offset_x_; // cx
  info->P[6] -= depth_ir_offset_y_; // cy

  /// @todo Could put this in projector frame so as to encode the baseline in P[3]
  return info;
}

sensor_msgs::CameraInfoPtr DriverNodelet::getProjectorCameraInfo(
    const ImageBuffer& image, ros::Time time) const {
  // The projector info is simply the depth info with the baseline encoded in the P matrix.
  // It's only purpose is to be the "right" camera info to the depth camera's "left" for
  // processing disparity images.
  sensor_msgs::CameraInfoPtr info = getDepthCameraInfo(image, time);
  // Tx = -baseline * fx
  info->P[3] = -device_->getBaseline() * info->P[0];
  return info;
}

void DriverNodelet::configCb(Config &config, uint32_t level)
{
  depth_ir_offset_x_ = config.depth_ir_offset_x;
  depth_ir_offset_y_ = config.depth_ir_offset_y;
  z_offset_mm_ = config.z_offset_mm;

  // We need this for the ASUS Xtion Pro
  OutputMode old_image_mode, image_mode, compatible_image_mode;
  if (device_->hasImageStream ())
  {
    old_image_mode = device_->getImageOutputMode ();

    // does the device support the new image mode?
    image_mode = mapConfigMode2OutputMode (config.image_mode);

    if (!device_->findCompatibleImageMode (image_mode, compatible_image_mode))
    {
      OutputMode default_mode = device_->getDefaultImageMode();
      NODELET_WARN("Could not find any compatible image output mode for %d. "
                   "Falling back to default image output mode %d.",
                    image_mode,
                    default_mode);

      config.image_mode = mapMode2ConfigMode(default_mode);
      image_mode = compatible_image_mode = default_mode;
    }
  }

  OutputMode old_depth_mode, depth_mode, compatible_depth_mode;
  old_depth_mode = device_->getDepthOutputMode();
  depth_mode = mapConfigMode2OutputMode (config.depth_mode);
  if (!device_->findCompatibleDepthMode (depth_mode, compatible_depth_mode))
  {
    OutputMode default_mode = device_->getDefaultDepthMode();
    NODELET_WARN("Could not find any compatible depth output mode for %d. "
                 "Falling back to default depth output mode %d.",
                  depth_mode,
                  default_mode);

    config.depth_mode = mapMode2ConfigMode(default_mode);
    depth_mode = compatible_depth_mode = default_mode;
  }

  // here everything is fine. Now make the changes
  if ( (device_->hasImageStream () && compatible_image_mode != old_image_mode) ||
       compatible_depth_mode != old_depth_mode)
  {
    // streams need to be reset!
    stopSynchronization();

    if (device_->hasImageStream () && compatible_image_mode != old_image_mode)
      device_->setImageOutputMode (compatible_image_mode);

    if (compatible_depth_mode != old_depth_mode)
      device_->setDepthOutputMode (compatible_depth_mode);

    startSynchronization ();
  }

  // @todo Run connectCb if registration setting changes
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
}

void DriverNodelet::startSynchronization()
{
  if (device_->isSynchronizationSupported() &&
      !device_->isSynchronized() &&
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
  OutputMode output_mode;

  output_mode = FREENECT_RESOLUTION_HIGH;
  mode2config_map_[output_mode] = Freenect_SXGA;
  config2mode_map_[Freenect_SXGA] = output_mode;

  output_mode = FREENECT_RESOLUTION_MEDIUM;
  mode2config_map_[output_mode] = Freenect_VGA;
  config2mode_map_[Freenect_VGA] = output_mode;
}

int DriverNodelet::mapMode2ConfigMode (const OutputMode& output_mode) const
{
  std::map<OutputMode, int>::const_iterator it = mode2config_map_.find (output_mode);

  if (it == mode2config_map_.end ())
  {
    NODELET_ERROR ("mode not be found");
    exit (-1);
  }
  else
    return it->second;
}

OutputMode DriverNodelet::mapConfigMode2OutputMode (int mode) const
{
  std::map<int, OutputMode>::const_iterator it = config2mode_map_.find (mode);
  if (it == config2mode_map_.end ())
  {
    NODELET_ERROR ("mode %d could not be found", mode);
    exit (-1);
  }
  else
    return it->second;
}

void DriverNodelet::watchDog (const ros::TimerEvent& event)
{
  bool timed_out = false;
  if (!rgb_time_stamp_.isZero() && device_->isImageStreamRunning()) {
    ros::Duration duration = ros::Time::now() - rgb_time_stamp_;
    timed_out = timed_out || duration.toSec() > time_out_;
  }
  if (!depth_time_stamp_.isZero() && device_->isDepthStreamRunning()) {
    ros::Duration duration = ros::Time::now() - depth_time_stamp_;
    timed_out = timed_out || duration.toSec() > time_out_;
  }
  if (!ir_time_stamp_.isZero() && device_->isIRStreamRunning()) {
    ros::Duration duration = ros::Time::now() - ir_time_stamp_;
    timed_out = timed_out || duration.toSec() > time_out_;
  }

  if (timed_out) {
    ROS_INFO("Device timed out. Flushing device.");
    if(subdevs_&FREENECT_DEVICE_CAMERA) device_->flushDeviceStreams();
  }

}

}

// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(freenect_camera::DriverNodelet, nodelet::Nodelet)
