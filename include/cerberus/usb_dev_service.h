#ifndef __CERBERUS_USB_DEV_SERVICE_H_
#define __CERBERUS_USB_DEV_SERVICE_H_

#include <string_view>

#include <boost/uuid/uuid.hpp>
#include <libusb-1.0/libusb.h>

namespace cerberus::services {

    class USBDeviceService {
      public:
        enum class status { ERROR = -1, RUNNING, CLOSED };
        USBDeviceService() = delete;

      private: //NOLINTBEGIN
        libusb_device** _devices;
        ssize_t _count{0};
        int _libusb_status;
        //NOLINTEND
    };

} // namespace cerberus::services

#endif