// Interface to a fake android logging system. We don't actually run android,
// we just have miniglog use this function so that it doesn't try to use stderr
// and abort().

#ifndef __ANDROID_LOG_H__
#define __ANDROID_LOG_H__

enum {
  ANDROID_LOG_FATAL,
  ANDROID_LOG_ERROR,
  ANDROID_LOG_WARN,
  ANDROID_LOG_INFO,
  ANDROID_LOG_DEBUG,
  ANDROID_LOG_VERBOSE,
};

extern void __android_log_write(int android_log_level,
                                const char *tag, const char *message);

#endif
