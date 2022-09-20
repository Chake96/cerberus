#ifndef __CERBERUS_C_CALLBACK_H_
#define __CERBERUS_C_CALLBACK_H_

#include <functional>
namespace cerberus::utilities {
    template <typename T>
    struct CCallback;

    template <typename Ret, typename... Params>
    struct CCallback<Ret(Params...)> {
        template <typename... Args>
        static Ret callback(Args... args) {
            return func(args...);
        }
        static std::function<Ret(Params...)> func;
    };

    // Initialize the static member.
    template <typename Ret, typename... Params>
    std::function<Ret(Params...)> CCallback<Ret(Params...)>::func;
} // namespace cerberus::utilities

#endif //__CERBERUS_CAMERAS_UVC_CAM_H_