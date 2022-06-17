#pragma once

//this is a copy from the Basics/standart.h
#ifndef BVUTILS_DLLIMPORT
#ifdef _MSC_VER
/* for use in C++ library header files when linking against dll */
#  ifdef BVUTILS_USE_DLL
#    define BVUTILS_DLLIMPORT __declspec(dllimport)
#  else
#    define BVUTILS_DLLIMPORT
#  endif
#else
/* other compilers don't need any import */
#  define BVUTILS_DLLIMPORT
#endif
#endif

#ifndef DLLExportMEPointCloud
#define DLLExportMEPointCloud BVUTILS_DLLIMPORT
#endif

#ifndef DEPRECATED 
#if defined(__GNUC__) || defined(__clang__)
#define DEPRECATED(message) __attribute__((deprecated(message)))
#elif defined(_MSC_VER)
#define DEPRECATED(message) __declspec(deprecated(message))
#else
#pragma message("WARNING: You need to implement DEPRECATED for this compiler")
#define DEPRECATED (message)
#endif
#endif
