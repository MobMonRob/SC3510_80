#ifndef ME_DECL_GLOBAL_H
#define ME_DECL_GLOBAL_H

#if defined _MSC_VER || defined __CYGWIN__
  #ifdef __GNUC__
    #define ME_DECL_EXPORT __attribute__ ((dllexport))
    #define ME_DECL_IMPORT __attribute__ ((dllimport))
  #else
    #define ME_DECL_EXPORT __declspec(dllexport)
    #define ME_DECL_IMPORT __declspec(dllimport)
  #endif
  #define ME_DECL_LOCAL
#else
  #if __GNUC__ >= 4
    #define ME_DECL_EXPORT     __attribute__((visibility("default")))
    #define ME_DECL_IMPORT     __attribute__((visibility("default")))
    #define ME_DECL_LOCAL      __attribute__((visibility("hidden")))
  #else
    #define ME_DECL_EXPORT
    #define ME_DECL_IMPORT
    #define ME_DECL_LOCAL
  #endif
#endif

#endif //ME_DECL_GLOBAL_H
