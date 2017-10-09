
// Simple performance monitoring, for measuring where all the time is spent.

#ifndef __TRACE_H__
#define __TRACE_H__

#include <string>

// Start a new trace.
void TraceStart();

// Measure the time taken during the lifetime of this class, labelling it with
// 'what'.
class Trace {
 public:
  explicit Trace(const char *what);
  ~Trace();
 private:
  int slot_;
};

// Stop the trace and get the full trace report.
void TraceReport(std::string *report);

// Catch bug where variable name is omitted, e.g. Trace("foo");
#define Trace(x) COMPILE_ASSERT(0, trace_decl_missing_var_name)

#endif
