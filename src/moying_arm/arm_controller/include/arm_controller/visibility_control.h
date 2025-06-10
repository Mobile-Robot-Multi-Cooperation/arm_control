#ifndef ARM_CONTROLLER__VISIBILITY_CONTROL_H_
#define ARM_CONTROLLER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ARM_CONTROLLER__VISIBILITY_EXPORT __attribute__((dllexport))
    #define ARM_CONTROLLER__VISIBILITY_IMPORT __attribute__((dllimport))
  #else
    #define ARM_CONTROLLER__VISIBILITY_EXPORT __declspec(dllexport)
    #define ARM_CONTROLLER__VISIBILITY_IMPORT __declspec(dllimport)
  #endif

  #ifdef ARM_CONTROLLER__VISIBILITY_BUILDING_DLL
    #define ARM_CONTROLLER__VISIBILITY_PUBLIC ARM_CONTROLLER__VISIBILITY_EXPORT
  #else
    #define ARM_CONTROLLER__VISIBILITY_PUBLIC ARM_CONTROLLER__VISIBILITY_IMPORT
  #endif

  #define ARM_CONTROLLER__VISIBILITY_PUBLIC_TYPE ARM_CONTROLLER__VISIBILITY_PUBLIC
  #define ARM_CONTROLLER__VISIBILITY_LOCAL
#else
  #define ARM_CONTROLLER__VISIBILITY_EXPORT __attribute__((visibility("default")))
  #define ARM_CONTROLLER__VISIBILITY_IMPORT

  #if __GNUC__ >= 4
    #define ARM_CONTROLLER__VISIBILITY_PUBLIC __attribute__((visibility("default")))
    #define ARM_CONTROLLER__VISIBILITY_LOCAL  __attribute__((visibility("hidden")))
  #else
    #define ARM_CONTROLLER__VISIBILITY_PUBLIC
    #define ARM_CONTROLLER__VISIBILITY_LOCAL
  #endif

  #define ARM_CONTROLLER__VISIBILITY_PUBLIC_TYPE
#endif

#endif  // ARM_CONTROLLER__VISIBILITY_CONTROL_H_