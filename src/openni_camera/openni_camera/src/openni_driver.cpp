/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011 2011 Willow Garage, Inc.
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
#include <openni_camera/openni_driver.h>
#include <openni_camera/openni_device_kinect.h>
#include <openni_camera/openni_device_primesense.h>
#include <openni_camera/openni_device_xtion.h>
#include <openni_camera/openni_device_oni.h>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <locale>
#include <cctype>
#include <map>

#ifndef _WIN32
#include <libusb-1.0/libusb.h>
#else
#include <boost/tokenizer.hpp>
#endif

using namespace std;
using namespace boost;

namespace openni_wrapper
{

OpenNIDriver::OpenNIDriver () throw (OpenNIException)
{
  // Initialize the Engine
  XnStatus status = context_.Init ();
  if (status != XN_STATUS_OK)
    THROW_OPENNI_EXCEPTION ("initialization failed. Reason: %s", xnGetStatusString (status));

  updateDeviceList ();
}

unsigned OpenNIDriver::updateDeviceList () throw ()
{
  // clear current list of devices
  device_context_.clear ();
  // clear maps
  bus_map_.clear ();
  serial_map_.clear ();
  connection_string_map_.clear ();

  // enumerate all devices
  static xn::NodeInfoList node_info_list;
  XnStatus status = context_.EnumerateProductionTrees (XN_NODE_TYPE_DEVICE, NULL, node_info_list);
  if (status != XN_STATUS_OK && node_info_list.Begin () != node_info_list.End ())
    THROW_OPENNI_EXCEPTION ("enumerating devices failed. Reason: %s", xnGetStatusString (status));
  else if (node_info_list.Begin () == node_info_list.End ())
    return 0; // no exception

  for (xn::NodeInfoList::Iterator nodeIt = node_info_list.Begin (); nodeIt != node_info_list.End (); ++nodeIt)
  {
    connection_string_map_[(*nodeIt).GetCreationInfo ()] = device_context_.size();
    device_context_.push_back (DeviceContext (*nodeIt));
  }

  // enumerate depth nodes
  static xn::NodeInfoList depth_nodes;
  status = context_.EnumerateProductionTrees (XN_NODE_TYPE_DEPTH, NULL, depth_nodes, NULL);
  if (status != XN_STATUS_OK)
    THROW_OPENNI_EXCEPTION ("enumerating depth generators failed. Reason: %s", xnGetStatusString (status));

  for (xn::NodeInfoList::Iterator nodeIt = depth_nodes.Begin (); nodeIt != depth_nodes.End (); ++nodeIt)
  {
    // check if to which device this node is assigned to
    for (xn::NodeInfoList::Iterator neededIt = (*nodeIt).GetNeededNodes ().Begin (); neededIt != (*nodeIt).GetNeededNodes ().End (); ++neededIt)
    {
      if ( connection_string_map_.count ((*neededIt).GetCreationInfo ()) )
      {
        unsigned device_index = connection_string_map_[(*neededIt).GetCreationInfo ()];
        device_context_[device_index].depth_node.reset (new xn::NodeInfo(*nodeIt));
      }
    }
  }

  // enumerate image nodes
  static xn::NodeInfoList image_nodes;
  static xn::EnumerationErrors enumeration_errors;
  status = context_.EnumerateProductionTrees (XN_NODE_TYPE_IMAGE, NULL, image_nodes, &enumeration_errors);
  if (status != XN_STATUS_OK)
  {
    // the asus pro doesn't produce images, so if the error is just "can't create node of the given type", then
    // we ignore it (that is what error code 65565 is). if this is some other error, for instance unable
    // to access the device due to permissions, then we throw an exception. -jbinney
    if((enumeration_errors.Begin()).Error() != 65565) {
      XnChar strError[1024];
      enumeration_errors.ToString(strError, 1024);
      THROW_OPENNI_EXCEPTION("enumerating image nodes failed. Reason: %s", strError);
    }
  }
  
  for (xn::NodeInfoList::Iterator nodeIt = image_nodes.Begin (); nodeIt != image_nodes.End (); ++nodeIt)
    {
      // check to which device this node is assigned to
      for (xn::NodeInfoList::Iterator neededIt = (*nodeIt).GetNeededNodes ().Begin (); neededIt != (*nodeIt).GetNeededNodes ().End (); ++neededIt)
        {
          if ( connection_string_map_.count ((*neededIt).GetCreationInfo ()) )
            {
              unsigned device_index = connection_string_map_[(*neededIt).GetCreationInfo ()];
              device_context_[device_index].image_node.reset (new xn::NodeInfo(*nodeIt));
            }
        }
    }
  
  // enumerate IR nodes
  static xn::NodeInfoList ir_nodes;
  status = context_.EnumerateProductionTrees (XN_NODE_TYPE_IR, NULL, ir_nodes, NULL);
  if (status != XN_STATUS_OK)
    THROW_OPENNI_EXCEPTION ("enumerating IR generators failed. Reason: %s", xnGetStatusString (status));

  for (xn::NodeInfoList::Iterator nodeIt = ir_nodes.Begin (); nodeIt != ir_nodes.End (); ++nodeIt)
  {
    // check if to which device this node is assigned to
    for (xn::NodeInfoList::Iterator neededIt = (*nodeIt).GetNeededNodes ().Begin (); neededIt != (*nodeIt).GetNeededNodes ().End (); ++neededIt)
    {
      if ( connection_string_map_.count ((*neededIt).GetCreationInfo ()) )
      {
        unsigned device_index = connection_string_map_[(*neededIt).GetCreationInfo ()];
        device_context_[device_index].ir_node.reset (new xn::NodeInfo(*nodeIt));
      }
    }
  }

#ifndef _WIN32
  // add context object for each found device
  for (unsigned deviceIdx = 0; deviceIdx < device_context_.size (); ++deviceIdx)
  {
    // register bus@address to the corresponding context object
    unsigned short vendor_id;
    unsigned short product_id;
    unsigned char bus;
    unsigned char address;
    sscanf (device_context_[deviceIdx].device_node.GetCreationInfo (), "%hx/%hx@%hhu/%hhu", &vendor_id, &product_id, &bus, &address);
    bus_map_ [bus][address] = deviceIdx;
  }

  // get additional info about connected devices like serial number, vendor name and prduct name
  getDeviceInfos ();
  // build serial number -> device index map
  for (unsigned deviceIdx = 0; deviceIdx < device_context_.size (); ++deviceIdx)
  {
    std::string serial_number = getSerialNumber (deviceIdx);
    if (!serial_number.empty ())
      serial_map_[serial_number] = deviceIdx;
  }

#endif

  // redundant, but needed for Windows right now and also for Xtion
  for (unsigned deviceIdx = 0; deviceIdx < device_context_.size (); ++deviceIdx)
  {
    unsigned short product_id;
    unsigned short vendor_id;

    getDeviceType(device_context_[deviceIdx].device_node.GetCreationInfo (), vendor_id, product_id );

#if _WIN32
    if (vendor_id == 0x45e)
    {
      strcpy ((char*)device_context_[deviceIdx].device_node.GetDescription().strVendor, "Microsoft");
      strcpy ((char*)device_context_[deviceIdx].device_node.GetDescription().strName, "Xbox NUI Camera");
    }
    else
#endif
    if (vendor_id == 0x1d27 && device_context_[deviceIdx].image_node.get () == 0)
    {
      strcpy ((char*)device_context_[deviceIdx].device_node.GetDescription().strVendor, "ASUS");
      strcpy ((char*)device_context_[deviceIdx].device_node.GetDescription().strName, "Xtion Pro");
    }
  }
  return (device_context_.size ());
}

void OpenNIDriver::stopAll () throw (OpenNIException)
{
  XnStatus status = context_.StopGeneratingAll ();
  if (status != XN_STATUS_OK)
    THROW_OPENNI_EXCEPTION ("stopping all streams failed. Reason: %s", xnGetStatusString (status));
}

OpenNIDriver::~OpenNIDriver () throw ()
{
  // no exception during destuctor
  try
  {
    stopAll ();
  }
  catch (...)
  {
  }

  context_.Shutdown ();
}

boost::shared_ptr<OpenNIDevice> OpenNIDriver::createVirtualDevice (const string& path, bool repeat, bool stream) const throw (OpenNIException)
{
  return boost::shared_ptr<OpenNIDevice> (new DeviceONI (context_, path, repeat, stream));
}

void OpenNIDriver::getPrimesenseSerial(xn::NodeInfo info, char* buffer, unsigned buf_size) const throw () {

        context_.CreateProductionTree(info);
        xn::Device device;

        if(info.GetInstance(device) != XN_STATUS_OK) {
            THROW_OPENNI_EXCEPTION ("couldn't get device instance for reading serial no.");
        }

        xn::DeviceIdentificationCapability d = device.GetIdentificationCap();

        d.GetSerialNumber(buffer,buf_size);

        device.Release();
}

boost::shared_ptr<OpenNIDevice> OpenNIDriver::getDeviceByIndex (unsigned index) const throw (OpenNIException)
{
  using namespace std;

  if (index >= device_context_.size ())
    THROW_OPENNI_EXCEPTION ("device index out of range. only %d devices connected but device %d requested.", device_context_.size (), index);
  boost::shared_ptr<OpenNIDevice> device = device_context_[index].device.lock ();
  if (!device)
  {
    unsigned short vendor_id;
    unsigned short product_id;
    getDeviceType (device_context_[index].device_node.GetCreationInfo (), vendor_id, product_id );

    if (vendor_id == 0x45e)
    {
      device = boost::shared_ptr<OpenNIDevice > (new DeviceKinect (context_, device_context_[index].device_node,
                                                                   *device_context_[index].image_node, *device_context_[index].depth_node,
                                                                   *device_context_[index].ir_node));
      device_context_[index].device = device;
    }
    else if (vendor_id == 0x1d27)
    {
      if (device_context_[index].image_node.get())
        device = boost::shared_ptr<OpenNIDevice > (new DevicePrimesense (context_, device_context_[index].device_node,
                                                                         *device_context_[index].image_node, *device_context_[index].depth_node,
                                                                         *device_context_[index].ir_node));
      else
        device = boost::shared_ptr<OpenNIDevice > (new DeviceXtionPro (context_, device_context_[index].device_node,
                                                                       *device_context_[index].depth_node, *device_context_[index].ir_node));
      device_context_[index].device = device;
    }
    else
    {
      THROW_OPENNI_EXCEPTION ("vendor %s (%s) known by primesense driver, but not by ros driver. Contact maintainer of the ros driver.",
                              getVendorName (index), vendor_id);
    }
  }
  return (device);
}

#ifndef _WIN32
boost::shared_ptr<OpenNIDevice> 
OpenNIDriver::getDeviceBySerialNumber (const std::string& serial_number) const throw (OpenNIException)
{
  std::map<std::string, unsigned>::const_iterator it = serial_map_.find (serial_number);

  if (it != serial_map_.end ())
  {
    return getDeviceByIndex (it->second);
  }

  THROW_OPENNI_EXCEPTION ("No device with serial number \'%s\' found", serial_number.c_str ());

  // because of warnings!!!
  return (boost::shared_ptr<OpenNIDevice > ((OpenNIDevice*)NULL));
}

boost::shared_ptr<OpenNIDevice> 
OpenNIDriver::getDeviceByAddress (unsigned char bus, unsigned char address) const throw (OpenNIException)
{
  std::map<unsigned char, std::map<unsigned char, unsigned> >::const_iterator busIt = bus_map_.find (bus);
  if (busIt != bus_map_.end ())
  {
    std::map<unsigned char, unsigned>::const_iterator devIt;
    // Interpret invalid address 0 as "open an arbitrary device on this bus"
    if (address == 0)
      devIt = busIt->second.begin ();
    else
      devIt = busIt->second.find (address);
    
    if (devIt != busIt->second.end ())
    {
      return getDeviceByIndex (devIt->second);
    }
  }

  THROW_OPENNI_EXCEPTION ("No device on bus: %d @ %d found", (int)bus, (int)address);

  // because of warnings!!!
  return (boost::shared_ptr<OpenNIDevice > ((OpenNIDevice*)NULL));
}

void 
OpenNIDriver::getDeviceInfos () throw ()
{
  libusb_context *context = NULL;
  int result;
  result = libusb_init (&context); //initialize a library session

  if (result < 0)
    return;

  libusb_device **devices;
  int count = libusb_get_device_list (context, &devices);
  if (count < 0)
    return;

  for (int devIdx = 0; devIdx < count; ++devIdx)
  {
    libusb_device* device = devices[devIdx];
    uint8_t busId = libusb_get_bus_number (device);
    std::map<unsigned char, std::map<unsigned char, unsigned> >::const_iterator busIt = bus_map_.find (busId);
    if (busIt == bus_map_.end ())
      continue;

    uint8_t address = libusb_get_device_address (device);
    std::map<unsigned char, unsigned>::const_iterator addressIt = busIt->second.find (address);
    if (addressIt == busIt->second.end ())
      continue;

    unsigned nodeIdx = addressIt->second;
    xn::NodeInfo& current_node = device_context_[nodeIdx].device_node;

    libusb_device_descriptor descriptor;
    result = libusb_get_device_descriptor (devices[devIdx], &descriptor);

    if (result < 0)
    {
      current_node.SetInstanceName ("");
    }
    else
    {
      libusb_device_handle* dev_handle;
      result = libusb_open (device, &dev_handle);
      if (result < 0)
      {
        current_node.SetInstanceName ("");
      }
      else
      {
        unsigned char buffer[1024];

        int len = libusb_get_string_descriptor_ascii (dev_handle, descriptor.iSerialNumber, buffer, 1024);
        if (len > 4)
          current_node.SetInstanceName ((char*)buffer);
        else
          current_node.SetInstanceName ("");

        libusb_close (dev_handle);
      }
    }
  }
  libusb_free_device_list (devices, 1);
  libusb_exit (context);
}
#endif

const char* OpenNIDriver::getSerialNumber (unsigned index) const throw ()
{
#ifndef _WIN32

    DeviceContext con = device_context_[index];
    const char* openni_serial = con.device_node.GetInstanceName ();

    if (strlen(openni_serial)>0 && strcmp(openni_serial, "Device1")) {
        return openni_serial;
    } else {
        char *primesense_serial = (char*)malloc(XN_MAX_NAME_LENGTH); // memleak
        getPrimesenseSerial(con.device_node, primesense_serial, XN_MAX_NAME_LENGTH);

        return primesense_serial;
    }

#else
  return "";
#endif
}

void OpenNIDriver::getDeviceType (const std::string& connectionString, unsigned short& vendorId, unsigned short& productId)
{
#if _WIN32
    // expected format: "\\?\usb#vid_[ID]&pid_[ID]#[SERIAL]#{GUID}"
    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
    boost::char_separator<char> separators("#&_");
    tokenizer tokens(connectionString, separators);

    unsigned int tokenIndex = 0;
    for (tokenizer::iterator tok_iter = tokens.begin(); tok_iter != tokens.end(); ++tok_iter, tokenIndex++)
    {
        std::string tokenValue = *tok_iter;

        switch(tokenIndex) {
            case 2:    // the vendor ID
                sscanf(tokenValue.c_str(), "%hx", &vendorId);
                break;
            case 4: // the product ID
                sscanf(tokenValue.c_str(), "%hx", &productId);
                break;
        }
    }
#else
    unsigned char bus;
    unsigned char address;
    sscanf (connectionString.c_str(), "%hx/%hx@%hhu/%hhu", &vendorId, &productId, &bus, &address);
#endif
}

const char* OpenNIDriver::getConnectionString (unsigned index) const throw ()
{
  return device_context_[index].device_node.GetCreationInfo ();
}

const char* OpenNIDriver::getVendorName (unsigned index) const throw ()
{
  return device_context_[index].device_node.GetDescription ().strVendor;
}

const char* OpenNIDriver::getProductName (unsigned index) const throw ()
{
  return device_context_[index].device_node.GetDescription ().strName;
}

unsigned short OpenNIDriver::getVendorID (unsigned index) const throw ()
{
  unsigned short vendor_id;
  unsigned short product_id;
#ifndef _WIN32
  unsigned char bus;
  unsigned char address;
  sscanf (device_context_[index].device_node.GetCreationInfo (), "%hx/%hx@%hhu/%hhu", &vendor_id, &product_id, &bus, &address);

#else
  getDeviceType (device_context_[index].device_node.GetCreationInfo (), vendor_id, product_id);
#endif
  return vendor_id;
}

unsigned short OpenNIDriver::getProductID (unsigned index) const throw ()
{
  unsigned short vendor_id;
  unsigned short product_id;
#ifndef _WIN32
  unsigned char bus;
  unsigned char address;
  sscanf (device_context_[index].device_node.GetCreationInfo (), "%hx/%hx@%hhu/%hhu", &vendor_id, &product_id, &bus, &address);

#else
  getDeviceType (device_context_[index].device_node.GetCreationInfo (), vendor_id, product_id);
#endif
  return product_id;
}

unsigned char OpenNIDriver::getBus (unsigned index) const throw ()
{
  unsigned char bus = 0;
#ifndef _WIN32
  unsigned short vendor_id;
  unsigned short product_id;
  unsigned char address;
  sscanf (device_context_[index].device_node.GetCreationInfo (), "%hx/%hx@%hhu/%hhu", &vendor_id, &product_id, &bus, &address);
#endif
  return bus;
}

unsigned char OpenNIDriver::getAddress (unsigned index) const throw ()
{
  unsigned char address = 0;
#ifndef _WIN32
  unsigned short vendor_id;
  unsigned short product_id;
  unsigned char bus;  
  sscanf (device_context_[index].device_node.GetCreationInfo (), "%hx/%hx@%hhu/%hhu", &vendor_id, &product_id, &bus, &address);
#endif
  return address;
}

OpenNIDriver::DeviceContext::DeviceContext (const xn::NodeInfo& device, xn::NodeInfo* image, xn::NodeInfo* depth, xn::NodeInfo* ir)
: device_node (device)
, image_node (image)
, depth_node (depth)
, ir_node (ir)
{
}

OpenNIDriver::DeviceContext::DeviceContext (const xn::NodeInfo& device)
: device_node (device)
, image_node ((xn::NodeInfo*)0)
, depth_node ((xn::NodeInfo*)0)
, ir_node ((xn::NodeInfo*)0)
{
}

OpenNIDriver::DeviceContext::DeviceContext (const DeviceContext& other)
: device_node (other.device_node)
, image_node (other.image_node)
, depth_node (other.depth_node)
, ir_node (other.ir_node)
, device (other.device)
{
}

} // namespace
