#ifndef MAGNET_PLUGIN__VISIBILITY_CONTROL_H_
#define MAGNET_PLUGIN__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MAGNET_PLUGIN_EXPORT __attribute__ ((dllexport))
    #define MAGNET_PLUGIN_IMPORT __attribute__ ((dllimport))
  #else
    #define MAGNET_PLUGIN_EXPORT __declspec(dllexport)
    #define MAGNET_PLUGIN_IMPORT __declspec(dllimport)
  #endif
  #ifdef MAGNET_PLUGIN_BUILDING_LIBRARY
    #define MAGNET_PLUGIN_PUBLIC MAGNET_PLUGIN_EXPORT
  #else
    #define MAGNET_PLUGIN_PUBLIC MAGNET_PLUGIN_IMPORT
  #endif
  #define MAGNET_PLUGIN_PUBLIC_TYPE MAGNET_PLUGIN_PUBLIC
  #define MAGNET_PLUGIN_LOCAL
#else
  #define MAGNET_PLUGIN_EXPORT __attribute__ ((visibility("default")))
  #define MAGNET_PLUGIN_IMPORT
  #if __GNUC__ >= 4
    #define MAGNET_PLUGIN_PUBLIC __attribute__ ((visibility("default")))
    #define MAGNET_PLUGIN_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MAGNET_PLUGIN_PUBLIC
    #define MAGNET_PLUGIN_LOCAL
  #endif
  #define MAGNET_PLUGIN_PUBLIC_TYPE
#endif

#endif  // MAGNET_PLUGIN__VISIBILITY_CONTROL_H_
