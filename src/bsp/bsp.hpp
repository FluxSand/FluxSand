#pragma once

#include <fcntl.h>
#include <gpiod.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <format>
#include <iostream>
#include <string>
#include <thread>
#include <type_traits>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef M_2PI
#define M_2PI 6.28318530717958647692f
#endif

#ifndef M_1G
#define M_1G 9.80665f
#endif

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#ifndef DEF2STR
#define TO_STR(_arg) #_arg
#define DEF2STR(_arg) TO_STR(_arg)
#endif

#ifndef UNUSED
#define UNUSED(_x) ((void)(_x))
#endif

#ifndef OFFSET_OF
#define OFFSET_OF(type, member) ((size_t)&((type *)0)->member)
#endif

#ifndef MEMBER_SIZE_OF
#define MEMBER_SIZE_OF(type, member) \
  (sizeof(std::remove_pointer_t<decltype(&((type *)0)->member)>))
#endif

#define CONTAINER_OF(ptr, type, member) \
  ((type *)((char *)(ptr) - OFFSET_OF(type, member)))

/**
 * Standard error codes used throughout the system.
 */
enum class ErrorCode : int8_t {
  OK = 0,
  FAILED = -1,
  INIT_ERR = -2,
  ARG_ERR = -3,
  STATE_ERR = -4,
  SIZE_ERR = -5,
  CHECK_ERR = -6,
  NOT_SUPPORT = -7,
  NOT_FOUND = -8,
  NO_RESPONSE = -9,
  NO_MEM = -10,
  NO_BUFF = -11,
  TIMEOUT = -12,
  EMPTY = -13,
  FULL = -14,
  BUSY = -15,
  PTR_NULL = -16,
  OUT_OF_RANGE = -17,
};
