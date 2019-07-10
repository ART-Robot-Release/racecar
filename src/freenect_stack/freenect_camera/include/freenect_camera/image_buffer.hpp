#ifndef IMAGE_BUFFER_6RYGHM2V
#define IMAGE_BUFFER_6RYGHM2V

#include <stdexcept>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>

#include <libfreenect/libfreenect.h>

namespace freenect_camera {

  const float RGB_FOCAL_LENGTH_SXGA = 1050;
  const float WIDTH_SXGA = 1280;

  /**
   * \class ImageBuffer
   *
   * \brief Holds an image buffer with all the metadata required to transmit the
   * image over ROS channels
   */
  struct ImageBuffer {
    boost::mutex mutex;
    boost::shared_array<unsigned char> image_buffer;
    int valid;
    freenect_frame_mode metadata;
    float focal_length;
    bool is_registered;
  };

  
  /**
   * Get RGB Focal length in pixels 
   */
  float getRGBFocalLength(int width) {
    float scale = width / WIDTH_SXGA;
    return RGB_FOCAL_LENGTH_SXGA * scale;
  }

  /**
   * Get Depth Focal length in pixels
   */
  float getDepthFocalLength(
      const freenect_registration& registration, int width) {

    float depth_focal_length_sxga = 
      registration.zero_plane_info.reference_distance /
      registration.zero_plane_info.reference_pixel_size;
    float scale = width / WIDTH_SXGA;
    return depth_focal_length_sxga * scale;
  }

  /**
   * Reallocate the video buffer if the video format or resolution changes
   */
  void allocateBufferVideo(
      ImageBuffer& buffer,
      const freenect_video_format& format,
      const freenect_resolution& resolution,
      const freenect_registration& registration) {

    // Obtain a lock on the buffer. This is mostly for debugging, as allocate
    // buffer should only be called when the buffer is not being used by the
    // freenect thread
    boost::lock_guard<boost::mutex> buffer_lock(buffer.mutex);

    // Deallocate the buffer incase an exception happens (the buffer should no
    // longer be valid)
    buffer.image_buffer.reset();

    switch (format) {
      case FREENECT_VIDEO_RGB:
      case FREENECT_VIDEO_BAYER:
      case FREENECT_VIDEO_YUV_RGB:
      case FREENECT_VIDEO_IR_8BIT:
      case FREENECT_VIDEO_IR_10BIT:
      case FREENECT_VIDEO_IR_10BIT_PACKED:
        switch (resolution) {
          case FREENECT_RESOLUTION_HIGH:
          case FREENECT_RESOLUTION_MEDIUM:
            buffer.metadata = 
              freenect_find_video_mode(resolution, format);
            if (!buffer.metadata.is_valid) {
              throw std::runtime_error("libfreenect: Invalid video fmt, res: " + 
                  boost::lexical_cast<std::string>(format) + "," +
                  boost::lexical_cast<std::string>(resolution));
            }
            break;
          default:
            throw std::runtime_error("libfreenect: Invalid video resolution: " +
                boost::lexical_cast<std::string>(resolution));
        }
        break;
      default:
        throw std::runtime_error("libfreenect: Invalid video format: " +
            boost::lexical_cast<std::string>(format));
    }

    // All is good, reallocate the buffer and calculate other pieces of info
    buffer.image_buffer.reset(new unsigned char[buffer.metadata.bytes]);
    switch(format) {
      case FREENECT_VIDEO_RGB:
      case FREENECT_VIDEO_BAYER:
      case FREENECT_VIDEO_YUV_RGB:
        buffer.focal_length = getRGBFocalLength(buffer.metadata.width);
        break;
      case FREENECT_VIDEO_IR_8BIT:
      case FREENECT_VIDEO_IR_10BIT:
      case FREENECT_VIDEO_IR_10BIT_PACKED:
        buffer.focal_length = getDepthFocalLength(registration, 
            buffer.metadata.width);
        break;
      default:
        throw std::runtime_error("libfreenect: shouldn't reach here");
    }
    buffer.is_registered = false;
  }

  /**
   * Reallocate the depth buffer if the depth format or resolution changes
   */
  void allocateBufferDepth(
      ImageBuffer& buffer,
      const freenect_depth_format& format,
      const freenect_resolution& resolution,
      const freenect_registration& registration) {

    // Obtain a lock on the buffer. This is mostly for debugging, as allocate
    // buffer should only be called when the buffer is not being used by the
    // freenect thread
    boost::lock_guard<boost::mutex> buffer_lock(buffer.mutex);

    // Deallocate the buffer incase an exception happens (the buffer should no
    // longer be valid)
    buffer.image_buffer.reset();

    switch (format) {
      case FREENECT_DEPTH_11BIT:
      case FREENECT_DEPTH_10BIT:
      case FREENECT_DEPTH_11BIT_PACKED:
      case FREENECT_DEPTH_10BIT_PACKED:
      case FREENECT_DEPTH_REGISTERED:
      case FREENECT_DEPTH_MM:
        switch (resolution) {
          case FREENECT_RESOLUTION_MEDIUM:
            buffer.metadata = 
              freenect_find_depth_mode(resolution, format);
            if (!buffer.metadata.is_valid) {
              throw std::runtime_error("libfreenect: Invalid depth fmt, res: " + 
                  boost::lexical_cast<std::string>(format) + "," +
                  boost::lexical_cast<std::string>(resolution));
            }
            break;
          default:
            throw std::runtime_error("libfreenect: Invalid depth resolution: " +
                boost::lexical_cast<std::string>(resolution));
        }
        break;
      default:
        throw std::runtime_error("libfreenect: Invalid depth format: " +
            boost::lexical_cast<std::string>(format));
    }

    // All is good, reallocate the buffer and calculate other pieces of info
    buffer.image_buffer.reset(new unsigned char[buffer.metadata.bytes]);
    switch(format) {
      case FREENECT_DEPTH_11BIT:
      case FREENECT_DEPTH_10BIT:
      case FREENECT_DEPTH_11BIT_PACKED:
      case FREENECT_DEPTH_10BIT_PACKED:
      case FREENECT_DEPTH_MM:
        buffer.focal_length = 
          getDepthFocalLength(registration, buffer.metadata.width);
        buffer.is_registered = false;
        break;
      case FREENECT_DEPTH_REGISTERED:
        buffer.focal_length = getRGBFocalLength(buffer.metadata.width);
        buffer.is_registered = true;
        break;
      default:
        throw std::runtime_error("libfreenect: shouldn't reach here");
    }
  }

  void fillImage(const ImageBuffer& buffer, void* data) {
    memcpy(data, buffer.image_buffer.get(), buffer.metadata.bytes);
  }

} /* end namespace freenect_camera */

#endif /* end of include guard: IMAGE_BUFFER_6RYGHM2V */
