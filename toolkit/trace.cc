
#include "trace.h"
#include "thread.h"
#include "mystring.h"
#include "myvector"

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
struct TraceInfo {                      // Information for each trace object
  const char *what;                     // Description of trace object
  long time_us;                         // Time taken, not counting sub levels
  long total_time_us;                   // Total time taken
  int level;                            // 'Indent' level of this trace
  bool finalized;                       // time fields have final values
};
static vector<TraceInfo> trace;
static int trace_level;                 // Current 'indent' level

void TraceStart() {
  MutexLock lock(&trace_mutex);
  STOPWATCH_START
  trace.clear();
  trace_level = 0;
}

Trace::Trace(const char *what) {
  MutexLock lock(&trace_mutex);
  slot_ = trace.size();
  trace.resize(trace.size() + 1);
  trace.back().what = what;
  trace.back().total_time_us = STOPWATCH_MICROSECONDS;
  trace.back().time_us = 0;
  trace.back().level = trace_level;
  trace.back().finalized = false;       // total_time_us is just start time
  trace_level++;
}

Trace::~Trace() {
  MutexLock lock(&trace_mutex);
  trace_level--;
  CHECK(trace_level >= 0);
  CHECK(slot_ < trace.size());  // Likely TraceStart() while trace objects live
  // Compute total time taken.
  trace[slot_].total_time_us = STOPWATCH_MICROSECONDS -
                               trace[slot_].total_time_us;
  trace[slot_].finalized = true;
  // To compute time_us, subtract the time taken by other trace objects
  // between this object's slot and the end of the trace array.
  trace[slot_].time_us = trace[slot_].total_time_us;
  for (int i = slot_ + 1; i < trace.size(); i++) {
    CHECK(trace[i].finalized);  // Likely using threads or recursion
    trace[slot_].time_us -= trace[i].time_us;
  }
}

void TraceReport(std::string *report) {
  MutexLock lock(&trace_mutex);
  if (trace.empty()) {
    report->clear();
    return;
  }
  long total = 0;
  for (int i = 0; i < trace.size(); i++) {
    total += trace[i].time_us;
  }
  StringPrintf(report, "Trace report. %%total %%parent (of subtree)"
               "\n%10.3fms 100.00%%         Total\n", double(total) / 1e3);
  vector<long> stack;
  stack.push_back(total);   // Stack of total times taken at different levels
  for (int i = 0; i < trace.size(); i++) {
    stack.resize(trace[i].level + 2);
    stack.back() = trace[i].total_time_us;
    StringAppendF(report, "%10.3fms %6.2f%% %6.2f%% %*c%s\n",
            double(trace[i].time_us) / 1e3,
            double(trace[i].time_us) / total * 100.0,
            double(trace[i].total_time_us) / stack[trace[i].level] * 100.0,
            (trace[i].level + 1) * 2, ' ', trace[i].what);
  }
}
