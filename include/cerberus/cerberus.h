#pragma once

#ifdef _WIN32
  #define cerberus_EXPORT __declspec(dllexport)
#else
  #define cerberus_EXPORT
#endif

cerberus_EXPORT void cerberus();
