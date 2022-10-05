#ifndef VDA5050_TEMPLATE_PACKAGE__VISIBILITY_CONTROL_H_
#define VDA5050_TEMPLATE_PACKAGE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define VDA5050_TEMPLATE_PACKAGE_EXPORT __attribute__ ((dllexport))
    #define VDA5050_TEMPLATE_PACKAGE_IMPORT __attribute__ ((dllimport))
  #else
    #define VDA5050_TEMPLATE_PACKAGE_EXPORT __declspec(dllexport)
    #define VDA5050_TEMPLATE_PACKAGE_IMPORT __declspec(dllimport)
  #endif
  #ifdef VDA5050_TEMPLATE_PACKAGE_BUILDING_LIBRARY
    #define VDA5050_TEMPLATE_PACKAGE_PUBLIC VDA5050_TEMPLATE_PACKAGE_EXPORT
  #else
    #define VDA5050_TEMPLATE_PACKAGE_PUBLIC VDA5050_TEMPLATE_PACKAGE_IMPORT
  #endif
  #define VDA5050_TEMPLATE_PACKAGE_PUBLIC_TYPE VDA5050_TEMPLATE_PACKAGE_PUBLIC
  #define VDA5050_TEMPLATE_PACKAGE_LOCAL
#else
  #define VDA5050_TEMPLATE_PACKAGE_EXPORT __attribute__ ((visibility("default")))
  #define VDA5050_TEMPLATE_PACKAGE_IMPORT
  #if __GNUC__ >= 4
    #define VDA5050_TEMPLATE_PACKAGE_PUBLIC __attribute__ ((visibility("default")))
    #define VDA5050_TEMPLATE_PACKAGE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define VDA5050_TEMPLATE_PACKAGE_PUBLIC
    #define VDA5050_TEMPLATE_PACKAGE_LOCAL
  #endif
  #define VDA5050_TEMPLATE_PACKAGE_PUBLIC_TYPE
#endif

#endif  // VDA5050_TEMPLATE_PACKAGE__VISIBILITY_CONTROL_H_
