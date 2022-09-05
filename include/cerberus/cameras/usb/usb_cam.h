#ifndef __CERBERUS_CAMERAS_USB_CAM_H_
#define __CERBERUS_CAMERAS_USB_CAM_H_

#include "cerberus/cameras/usb/utils.h"
#include <cerberus/cameras/utils.h>

#include <string_view>

namespace cerberus::cameras::usb {

    class USBCamera {
      public: //methods
        USBCamera() = delete;
        USBCamera(const std::string_view& name, const std::string_view& usb_desc, cameras::codecs codec = codecs::H264_AVC);

      private: //vars
        const std::string_view _simple_name;
        const std::string_view _usb_descriptor;
        const cameras::codecs _codec;
    };

} // namespace cerberus::cameras::usb

#endif //__CERBERUS_CAMERAS_USB_CAM_H_