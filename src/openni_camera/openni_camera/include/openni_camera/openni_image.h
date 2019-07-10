/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011 Willow Garage, Inc.
 *    Suat Gedikli <gedikli@willowgarage.com>
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
#ifndef __OPENNI_IMAGE__
#define __OPENNI_IMAGE__

#include <XnCppWrapper.h>
#include "openni_exception.h"
#include <boost/shared_ptr.hpp>

namespace openni_wrapper
{

/**
 * @brief Image class containing just a reference to image meta data. Thus this class
 * just provides an interface to fill a RGB or Grayscale image buffer.
 * @author Suat Gedikli
 * @date 02.january 2011
 * @param image_meta_data
 */
class Image
{
public:
  typedef boost::shared_ptr<Image> Ptr;
  typedef boost::shared_ptr<const Image> ConstPtr;

  typedef enum
  {
    BAYER_GRBG,
    YUV422,
    RGB
  } Encoding;

  inline Image (boost::shared_ptr<xn::ImageMetaData> image_meta_data) throw ();
  inline virtual ~Image () throw ();
  virtual bool isResizingSupported (unsigned input_width, unsigned input_height,
                                    unsigned output_width, unsigned output_height) const = 0;
  virtual void fillRGB (unsigned width, unsigned height, unsigned char* rgb_buffer,
                        unsigned rgb_line_step = 0) const throw (OpenNIException) = 0;

  virtual Encoding getEncoding () const = 0;

  inline void 
  fillRaw (unsigned char* rgb_buffer) const throw (OpenNIException)
  {
    memcpy (rgb_buffer, image_md_->WritableData(), image_md_->DataSize ());
  }

  virtual void fillGrayscale (unsigned width, unsigned height, unsigned char* gray_buffer,
                              unsigned gray_line_step = 0) const throw (OpenNIException) = 0;

  inline unsigned getWidth () const throw ();
  inline unsigned getHeight () const throw ();
  inline unsigned getFrameID () const throw ();
  inline unsigned long getTimeStamp () const throw ();
  inline const xn::ImageMetaData& getMetaData () const throw ();
  inline const boost::shared_ptr<xn::ImageMetaData> getMetaDataPtr () const;

protected:
  boost::shared_ptr<xn::ImageMetaData> image_md_;
};

Image::Image (boost::shared_ptr<xn::ImageMetaData> image_meta_data) throw ()
: image_md_ (image_meta_data)
{
}

Image::~Image () throw ()
{
}

unsigned Image::getWidth () const throw ()
{
  return image_md_->XRes ();
}

unsigned Image::getHeight () const throw ()
{
  return image_md_->YRes ();
}

unsigned Image::getFrameID () const throw ()
{
  return image_md_->FrameID ();
}

unsigned long Image::getTimeStamp () const throw ()
{
  return (unsigned long) image_md_->Timestamp ();
}

const xn::ImageMetaData& Image::getMetaData () const throw ()
{
	return *image_md_;
}

const boost::shared_ptr<xn::ImageMetaData> Image::getMetaDataPtr () const
{
  return image_md_;
}
} // namespace
#endif //__OPENNI_IMAGE__
