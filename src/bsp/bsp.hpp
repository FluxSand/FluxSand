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

#ifndef GRAVITY
#define GRAVITY 9.84f
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
