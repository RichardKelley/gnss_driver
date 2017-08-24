// Copied from Apollo's GNSS driver.

// Commonly-used macro definitions. Some of them are copied from
// https://chromium.googlesource.com/chromium/src/base/+/master/macros.h

#ifndef MODULES_DRIVERS_GNSS_INCLUDE_UTIL_MACROS_H_
#define MODULES_DRIVERS_GNSS_INCLUDE_UTIL_MACROS_H_

#include <cstddef>

// Put this in the declarations for a class to be uncopyable.
#define DISABLE_COPY(TypeName) TypeName(const TypeName &) = delete

// Put this in the declarations for a class to be unassignable.
#define DISABLE_ASSIGN(TypeName) void operator=(const TypeName &) = delete

// A macro to disallow the copy constructor and operator= functions.
// This should be used in the private: declarations for a class.
#define DISABLE_COPY_AND_ASSIGN(TypeName) \
  DISABLE_COPY(TypeName);                 \
  DISABLE_ASSIGN(TypeName)

// A macro to disallow all the implicit constructors, namely the
// default constructor, copy constructor and operator= functions.
//
// This should be used in the private: declarations for a class
// that wants to prevent anyone from instantiating it. This is
// especially useful for classes containing only static methods.
#define DISABLE_IMPLICIT_CONSTRUCTORS(TypeName) \
  TypeName() = delete;                          \
  DISABLE_COPY_AND_ASSIGN(TypeName)

// Creates a thread-safe singleton.
#define MAKE_SINGLETON(TypeName) \
 public:                         \
  static TypeName *instance() {  \
    static TypeName instance;    \
    return &instance;            \
  }                              \
                                 \
 private:                        \
  DISABLE_COPY_AND_ASSIGN(TypeName)

namespace gnss_driver {

// array_size(a) returns the number of elements in a.
template <class T, size_t N>
constexpr size_t array_size(T (&)[N]) {
  return N;
}

} 

#endif  // MODULES_DRIVERS_GNSS_INCLUDE_UTIL_MACROS_H_
