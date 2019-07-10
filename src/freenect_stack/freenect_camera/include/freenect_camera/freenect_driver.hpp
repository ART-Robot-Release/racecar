#ifndef FREENECT_DRIVER_K8EEAIBB
#define FREENECT_DRIVER_K8EEAIBB

#include <libfreenect/libfreenect.h>
#include <freenect_camera/freenect_device.hpp>

namespace freenect_camera {

  class FreenectDriver {

    public:


      static FreenectDriver& getInstance(freenect_device_flags subdevs) {
        static FreenectDriver instance(subdevs);
        return instance;
      }

      static FreenectDriver& getInstance() {
        static FreenectDriver instance;
        return instance;
      }
      void shutdown() {
        thread_running_ = false;
        freenect_thread_->join();
        if (device_)
          device_->shutdown();
        device_.reset();
        freenect_shutdown(driver_);
      }

      void updateDeviceList() {
        device_serials_.clear();
        freenect_device_attributes* attr_list;
        freenect_device_attributes* item;
        freenect_list_device_attributes(driver_, &attr_list);
        for (item = attr_list; item != NULL; item = item->next) {
          device_serials_.push_back(std::string(item->camera_serial));
        }
        freenect_free_device_attributes(attr_list);
      }

      unsigned getNumberDevices() {
        return device_serials_.size();
      }

      /** Unsupported */
      unsigned getBus(unsigned device_idx) {
        return 0;
      }

      /** Unsupported */
      unsigned getAddress(unsigned device_idx) {
        return 0;
      }

      const char* getProductName(unsigned device_idx) {
        return PRODUCT_NAME.c_str();
      }

      unsigned getProductID(unsigned device_idx) {
        return PRODUCT_ID;
      }

      const char* getVendorName(unsigned device_idx) {
        return VENDOR_NAME.c_str();
      }

      unsigned getVendorID(unsigned device_idx) {
        return VENDOR_ID;
      }

      const char* getSerialNumber(unsigned device_idx) {
        if (device_idx < getNumberDevices())
          return device_serials_[device_idx].c_str();
        throw std::runtime_error("libfreenect: device idx out of range");
      }

      boost::shared_ptr<FreenectDevice> getDeviceByIndex(unsigned device_idx) {
        return getDeviceBySerialNumber(std::string(getSerialNumber(device_idx)));
      }

      boost::shared_ptr<FreenectDevice> getDeviceBySerialNumber(std::string serial) {
        device_.reset(new FreenectDevice(driver_, serial));
        // start freenect thread now that we have device
        thread_running_ = true;
        freenect_thread_.reset(new boost::thread(boost::bind(&FreenectDriver::process, this)));
        return device_;
      }

      boost::shared_ptr<FreenectDevice> getDeviceByAddress(unsigned bus, unsigned address) {
        throw std::runtime_error("[ERROR] libfreenect does not support searching for device by bus/address");
      }

      void process() {
        while (thread_running_) {
          timeval t;
          t.tv_sec = 0;
          t.tv_usec = 10000;
          if (freenect_process_events_timeout(driver_, &t) < 0)
            throw std::runtime_error("freenect_process_events error");
          if (device_)
            device_->executeChanges();
        }
      }

      void enableDebug() {
        freenect_set_log_level(driver_, FREENECT_LOG_SPEW);
      }

    private:
      FreenectDriver() {
        freenect_init(&driver_, NULL);  //init driver_ handle
        freenect_set_log_level(driver_, FREENECT_LOG_FATAL); // Prevent's printing stuff to the screen
        freenect_select_subdevices(driver_, (freenect_device_flags)(FREENECT_DEVICE_CAMERA));
        thread_running_ = false;
        subdevs_=(freenect_device_flags)(FREENECT_DEVICE_CAMERA);
      }
      //add support for select subdevs such as motor or audio
      FreenectDriver(freenect_device_flags subdevs) {
        freenect_init(&driver_, NULL);  //init driver_ handle
        freenect_set_log_level(driver_, FREENECT_LOG_FATAL); // Prevent's printing stuff to the screen
        //open camera as default if not select any subdevs
        if(0==(subdevs&(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA | FREENECT_DEVICE_AUDIO)))
        {
          ROS_INFO("Did not select any subdevs,open AUDIO as default.\n");
          subdevs=(freenect_device_flags)(FREENECT_DEVICE_AUDIO);
        }
        freenect_select_subdevices(driver_, subdevs);
        thread_running_ = false;
        subdevs_=subdevs;
      }
      freenect_context* driver_;
      std::vector<std::string> device_serials_;
      boost::shared_ptr<boost::thread> freenect_thread_;
      boost::shared_ptr<FreenectDevice> device_;
      freenect_device_flags subdevs_;
      bool thread_running_;
  };

}

#endif /* end of include guard: FREENECT_DRIVER_K8EEAIBB */
