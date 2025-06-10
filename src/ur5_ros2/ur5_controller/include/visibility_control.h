#ifndef UR5_CONTROLLER__VISIBILITY_CONTROL_H_
#define UR5_CONTROLLER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define UR5_CONTROLLER__VISIBILITY_EXPORT __attribute__((dllexport))
    #define UR5_CONTROLLER__VISIBILITY_IMPORT __attribute__((dllimport))
  #else
    #define UR5_CONTROLLER__VISIBILITY_EXPORT __declspec(dllexport)
    #define UR5_CONTROLLER__VISIBILITY_IMPORT __declspec(dllimport)
  #endif

  #ifdef UR5_CONTROLLER__VISIBILITY_BUILDING_DLL
    #define UR5_CONTROLLER__VISIBILITY_PUBLIC UR5_CONTROLLER__VISIBILITY_EXPORT
  #else
    #define UR5_CONTROLLER__VISIBILITY_PUBLIC UR5_CONTROLLER__VISIBILITY_IMPORT
  #endif

  #define UR5_CONTROLLER__VISIBILITY_PUBLIC_TYPE UR5_CONTROLLER__VISIBILITY_PUBLIC
  #define UR5_CONTROLLER__VISIBILITY_LOCAL
#else
  #define UR5_CONTROLLER__VISIBILITY_EXPORT __attribute__((visibility("default")))
  #define UR5_CONTROLLER__VISIBILITY_IMPORT

  #if __GNUC__ >= 4
    #define UR5_CONTROLLER__VISIBILITY_PUBLIC __attribute__((visibility("default")))
    #define UR5_CONTROLLER__VISIBILITY_LOCAL __attribute__((visibility("hidden")))
  #else
    #define UR5_CONTROLLER__VISIBILITY_PUBLIC
    #define UR5_CONTROLLER__VISIBILITY_LOCAL
  #endif

  #define UR5_CONTROLLER__VISIBILITY_PUBLIC_TYPE
#endif

#endif  // UR5_CONTROLLER__VISIBILITY_CONTROL_H_