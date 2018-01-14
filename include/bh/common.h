//==================================================
// common.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Mar 15, 2017
//==================================================

#pragma once

#include <stdexcept>
#include <string>
#include <chrono>
#include <thread>
#include <algorithm>
#include <iostream>

#if !BH_NO_STACKTRACE
#include <signal.h>
#include <boost/stacktrace.hpp>
#include <boost/exception/all.hpp>
#endif

#ifndef _MSC_VER
#define NOEXCEPT noexcept
#else
#define NOEXCEPT
#endif

// Easy value printer
#define BH_PRINT_VALUE(x) std::cout << #x << "=" << (x) << std::endl

// Marco for function name and line number as a string
#define BH_FUNCTION_LINE_STRING (std::string(__FILE__) + " [" + std::string(__FUNCTION__) + ":" + std::to_string(__LINE__) + "]")

// Exceptions
#define ANNOTATE_EXC(type, s) type (std::string(BH_FUNCTION_LINE_STRING).append(": ").append(s))

#ifndef BH_EXCEPTION
  #define BH_EXCEPTION(s) bh::Exception(std::string(BH_FUNCTION_LINE_STRING).append(": ").append(s).c_str())
#endif

// Warning and error output
#if !defined(MLIB_WARNING) && BH_MLIB_COMPATIBILITY
  #define MLIB_WARNING(s) bh::warningFunction(std::string(BH_FUNCTION_LINE_STRING) + ": " + std::string(s))
#endif

#if !defined(MLIB_ERROR) && BH_MLIB_COMPATIBILITY
  #define MLIB_ERROR(s) bh::errorFunction(std::string(BH_FUNCTION_LINE_STRING) + ": " + std::string(s))
#endif

#define BH_DEBUG_BREAK bh::debugBreakFunction()

// Assertion macros
#if !BH_NO_ASSERT
  #define BH_ASSERT_STR(b, s) { if (!(b)) { bh::assertMessage(std::string(BH_FUNCTION_LINE_STRING) + ": " + std::string(s)); } }
  #define BH_ASSERT(b) { if (!(b)) { bh::assertMessage(BH_FUNCTION_LINE_STRING); } }
  #if defined(DEBUG) || defined(_DEBUG)
    #define BH_ASSERT_DBG_STR(b, s) { if (!(b)) { bh::assertMessage(std::string(BH_FUNCTION_LINE_STRING) + ": " + std::string(s)); } }
    #define BH_ASSERT_DBG(b) { if (!(b)) { bh::assertMessage(BH_FUNCTION_LINE_STRING); } }
  #else
    #define BH_ASSERT_DBG_STR(b, s)
    #define BH_ASSERT_DBG(b)
  #endif
#endif

// Safe delete and free macros
#ifndef SAFE_DELETE
    #define SAFE_DELETE(p)       { if (p != nullptr) { delete (p); (p) = nullptr; } }
#endif

#ifndef SAFE_DELETE_ARRAY
    #define SAFE_DELETE_ARRAY(p) { if (p != nullptr) { delete[] (p);   (p) = nullptr; } }
#endif

#ifndef SAFE_FREE
    #define SAFE_FREE(p) { if (p != nullptr) { std::free(p);   (p) = nullptr; } }
#endif


namespace bh {

class Exception : public std::exception {
public:
  explicit Exception(const std::string& what)
      : std::exception() {
      what_ = what;
  }

  explicit Exception(const char* what)
      : std::exception() {
      what_ = std::string(what);
  }

  const char* what() const NOEXCEPT {
      return what_.c_str();
  }

  template <typename Stream>
  Stream& printStacktrace() const NOEXCEPT {
    printStacktrace(std::cerr);
  }

#if BH_NO_STACKTRACE
  template <typename Stream>
  Stream& printStacktrace(Stream& out) const NOEXCEPT {
    out << "WARNING: Libbh was compiled without stack trace support." << std::endl;
    return out;
  }

  bool hasStacktrace() const NOEXCEPT {
    return false;
  }
#else
  template <typename Stream>
  Stream& printStacktrace(Stream& out) const NOEXCEPT {
    out << stacktrace_ << std::endl;
    return out;
  }

  bool hasStacktrace() const NOEXCEPT {
    return true;
  }

  boost::stacktrace::stacktrace getStacktrace() const NOEXCEPT {
    return stacktrace_;
  }
#endif

private:
    std::string what_;
#if !BH_NO_STACKTRACE
    boost::stacktrace::stacktrace stacktrace_;
#endif
};

template <typename _CharT>
std::basic_ostream<_CharT>& operator<<(std::basic_ostream<_CharT>& out, const Exception& exc) {
  out << "Exception occured: " << exc.what() << std::endl;
  if (exc.hasStacktrace()) {
    out << "Stacktrace:" << std::endl;
    exc.printStacktrace(out);
  }
  return out;
}

class Error : public Exception {
public:
  explicit Error(const std::string& what)
    : Exception(what) {}

  explicit Error(const char* what)
    : Exception(what) {}
};

template <typename Stream>
Stream& printStacktrace(Stream& out);
void printStacktrace();

void registerExitStacktraceFunction();
void registerAbortStacktraceFunction();
void registerSegfaultStacktraceFunction();

void debugBreakFunction();

void warningFunction(const std::string& description);

void errorFunction(const std::string& description);

void assertFunction(bool predicate, const std::string& description);

void assertMessage(const std::string& description);

}

#include <csignal>
#include <iostream>

namespace {

#if !BH_NO_STACKTRACE
inline void signal_handler_stacktrace(int signum) {
  ::signal(signum, SIG_DFL);
  bh::printStacktrace();
  ::raise(signum);
}
#endif

}

namespace bh {

// -------------------------
// Implementation
// -------------------------

inline void printStacktrace() {
  printStacktrace(std::cerr);
}

inline void registerExitStacktraceFunction() {
  registerAbortStacktraceFunction();
  registerSegfaultStacktraceFunction();
}

#if BH_NO_STACKTRACE
template <typename Stream>
Stream& printStacktrace(Stream& out) {
  std::cout << "WARNING: Cannot register exit stacktrace handler. "
            << "Libbh was compiled without stack trace support." << std::endl;
    return out;
}

inline void registerAbortStacktraceFunction() {
  std::cout << "WARNING: Cannot register SIGABRT stacktrace handler. "
            << "Libbh was compiled without stack trace support." << std::endl;
}

inline void registerSegfaultStacktraceFunction() {
  std::cout << "WARNING: Cannot register SIGSEGV stacktrace handler. "
            << "Libbh was compiled without stack trace support." << std::endl;
}
#else
template <typename Stream>
Stream& printStacktrace(Stream& out) {
  out << boost::stacktrace::stacktrace() << std::endl;
  return out;
}

inline void registerAbortStacktraceFunction() {
  ::signal(SIGABRT, &signal_handler_stacktrace);
}

inline void registerSegfaultStacktraceFunction() {
  ::signal(SIGSEGV, &signal_handler_stacktrace);
}
#endif


#if __GNUC__ && !__CUDACC__
#pragma GCC push_options
#pragma GCC optimize("O0")
#endif
inline void debugBreakFunction() {
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
  __debugbreak();
#else
  std::raise(SIGINT);
#endif
}
#if __GNUC__ && !__CUDACC__
#pragma GCC pop_options
#endif

inline void warningFunction(const std::string& description) {
  std::cout << "WARNING: " << description << std::endl;
}

inline void errorFunction(const std::string &description)
{
  std::cerr << "ERROR: " << description << std::endl;
#if _DEBUG || BH_DEBUG || BH_ASSERT_BREAK
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
  __debugbreak();
#else
  std::raise(SIGINT);
#endif
#endif
}

inline void assertFunction(bool predicate, const std::string& description) {
  if(!predicate) {
    assertMessage(description);
  }
}

inline void assertMessage(const std::string& description) {
  std::cerr << "Assertion failed. " << description << std::endl;
#if _DEBUG || BH_DEBUG || BH_ASSERT_BREAK
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
  __debugbreak();
#else
  std::raise(SIGINT);
#endif
#else
  throw BH_EXCEPTION(std::string("Assertion failed: ") + description);
#endif
}

}
