// Copyright (C) 2014-2020 Russell Smith.
//
// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.

#include "trace.h"
#include "thread.h"
#include "mystring.h"
#include "myvector"
#include <map>

using std::vector;

//***************************************************************************
// Performance monitoring.

#undef Trace

#ifdef __TOOLKIT_WXWINDOWS__
#include "stdwx.h"
static wxStopWatch stopwatch;
#define STOPWATCH_START stopwatch.Start();
#define STOPWATCH_MICROSECONDS (stopwatch.TimeInMicro().ToLong())
#endif

#ifdef QT_CORE_LIB
#include <QElapsedTimer>
static QElapsedTimer stopwatch;
#define STOPWATCH_START stopwatch.start();
#define STOPWATCH_MICROSECONDS (stopwatch.nsecsElapsed() / 1000)
#endif

static Mutex trace_mutex;               // Protects variables below

// Information for each trace object.
struct TraceInfo {
  const char *what;                     // Description of trace object
  long time_us;                         // Time taken, not counting sub levels
  long total_time_us;                   // Total time taken
  int level;                            // 'Indent' level of this trace
  bool finalized;                       // time fields have final values
};

// A stack of trace objects, one stack for each thread.
struct TraceStack {
  vector<TraceInfo> trace;
  int trace_level;                      // Current 'indent' level
  TraceStack() : trace_level(0) {}
};
static std::map<CurrentThreadID_t, TraceStack> stack;

void TraceStart() {
  MutexLock lock(&trace_mutex);
  STOPWATCH_START
  auto &s = stack[GetCurrentThreadID()];
  s.trace.clear();
  s.trace_level = 0;
}

Trace::Trace(const char *what) {
  MutexLock lock(&trace_mutex);
  auto &s = stack[GetCurrentThreadID()];
  slot_ = s.trace.size();
  s.trace.resize(s.trace.size() + 1);
  s.trace.back().what = what;
  s.trace.back().total_time_us = STOPWATCH_MICROSECONDS;
  s.trace.back().time_us = 0;
  s.trace.back().level = s.trace_level;
  s.trace.back().finalized = false;       // total_time_us is just start time
  s.trace_level++;
}

Trace::~Trace() {
  MutexLock lock(&trace_mutex);
  auto &s = stack[GetCurrentThreadID()];
  s.trace_level--;
  CHECK(s.trace_level >= 0);
  CHECK(slot_ < s.trace.size()); // Likely TraceStart() while trace objects live
  // Compute total time taken.
  s.trace[slot_].total_time_us = STOPWATCH_MICROSECONDS -
                                 s.trace[slot_].total_time_us;
  s.trace[slot_].finalized = true;
  // To compute time_us, subtract the time taken by other trace objects
  // between this object's slot and the end of the trace array.
  s.trace[slot_].time_us = s.trace[slot_].total_time_us;
  for (int i = slot_ + 1; i < s.trace.size(); i++) {
    CHECK(s.trace[i].finalized);  // Likely using threads or recursion
    s.trace[slot_].time_us -= s.trace[i].time_us;
  }
}

void TraceReport(std::string *report) {
  MutexLock lock(&trace_mutex);
  auto &s = stack[GetCurrentThreadID()];
  if (s.trace.empty()) {
    report->clear();
    return;
  }
  long total = 0;
  for (int i = 0; i < s.trace.size(); i++) {
    total += s.trace[i].time_us;
  }
  StringPrintf(report, "Trace report. %%total %%parent (of subtree)"
               "\n%10.3fms 100.00%%         Total\n", double(total) / 1e3);
  vector<long> stack;
  stack.push_back(total);   // Stack of total times taken at different levels
  for (int i = 0; i < s.trace.size(); i++) {
    stack.resize(s.trace[i].level + 2);
    stack.back() = s.trace[i].total_time_us;
    StringAppendF(report, "%10.3fms %6.2f%% %6.2f%% %*c%s\n",
            double(s.trace[i].time_us) / 1e3,
            double(s.trace[i].time_us) / total * 100.0,
            double(s.trace[i].total_time_us) / stack[s.trace[i].level] * 100.0,
            (s.trace[i].level + 1) * 2, ' ', s.trace[i].what);
  }
}
