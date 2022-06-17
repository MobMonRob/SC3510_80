#pragma once

#include <memory>
#include <vector>

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

#ifndef DLLExportMEDataTypes
#define DLLExportMEDataTypes BVUTILS_DLLIMPORT
#endif


#define DEFINE_MEDATATYPES_PTRS( classname )       \
    using Ptr = std::shared_ptr<classname>;        \
    using CPtr = std::shared_ptr<classname const>; \
    using UPtr = std::unique_ptr<classname>;        

#define DEFINE_MEDATATYPES_EXTERN_PTRS( classname )             \
    using classname ## Ptr = std::shared_ptr<classname>;        \
    using classname ## CPtr = std::shared_ptr<classname const>; \

