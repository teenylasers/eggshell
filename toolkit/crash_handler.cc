
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <execinfo.h>

// Print a message for a value of si_code
static const char *DecodeSICode(int signum, int si_code) {
  if (signum == SIGILL) {
    if (si_code == ILL_ILLOPC) return "illegal opcode";
    if (si_code == ILL_ILLTRP) return "illegal trap";
    if (si_code == ILL_PRVOPC) return "privileged opcode";
    if (si_code == ILL_ILLOPN) return "illegal operand";
    if (si_code == ILL_ILLADR) return "illegal addressing mode";
    if (si_code == ILL_PRVREG) return "privileged register";
    if (si_code == ILL_COPROC) return "coprocessor error";
    if (si_code == ILL_BADSTK) return "internal stack error";
  } else if (signum == SIGFPE) {
    if (si_code == FPE_FLTDIV) return "floating point divide by zero";
    if (si_code == FPE_FLTOVF) return "floating point overflow";
    if (si_code == FPE_FLTUND) return "floating point underflow";
    if (si_code == FPE_FLTRES) return "floating point inexact result";
    if (si_code == FPE_FLTINV) return "invalid floating point operation";
    if (si_code == FPE_FLTSUB) return "subscript out of range";
    if (si_code == FPE_INTDIV) return "integer divide by zero";
    if (si_code == FPE_INTOVF) return "integer overflow";
  } else if (signum == SIGSEGV) {
    if (si_code == SEGV_MAPERR) return "address not mapped to object";
    if (si_code == SEGV_ACCERR) return "invalid permission for mapped object";
  } else if (signum == SIGBUS) {
    if (si_code == BUS_ADRALN) return "Invalid address alignment";
    if (si_code == BUS_ADRERR) return "Nonexistent physical address";
    if (si_code == BUS_OBJERR) return "Object-specific HW error";
  } else if (signum == SIGTRAP) {
    if (si_code == TRAP_BRKPT) return "Process breakpoint";
    if (si_code == TRAP_TRACE) return "Process trace trap";
  }
  return 0;
}

static void WriteHex16(size_t n) {
  for (int i = 15; i >= 0; i--) {
    size_t m = n >> ((i * 4)) & 0xf;
    char c = (m <= 9) ? m + '0' : m - 10 + 'a';
    write(1, &c, 1);
  }
}

static void SignalHandler(int signum, siginfo_t *info, void *) {
  // Try not to allocate any heap memory in case the heap is trashed.
  const char *msg1 = "\n\e[35mSIGNAL RECEIVED: ";
  const char *msg2 = "\nBACKTRACE:\n";
  const char *msg3 = "\e[0m\n";
  write(1, msg1, strlen(msg1));
  write(1, sys_siglist[signum], strlen(sys_siglist[signum]));
  if (info) {
    const char *description = DecodeSICode(signum, info->si_code);
    if (description) {
      write(1, ": ", 2);
      write(1, description, strlen(description));
    }
  }
  if (signum == SIGILL || signum == SIGFPE || signum == SIGSEGV ||
      signum == SIGBUS) {
    const char *msg = " (si_addr = 0x";
    write(1, msg, strlen(msg));
    WriteHex16(reinterpret_cast<size_t>(info->si_addr));
    write(1, ")", 1);
  }
  write(1, msg2, strlen(msg2));
  void *callstack[128];
  int frames = backtrace(callstack, sizeof(callstack) / sizeof(void*));
  backtrace_symbols_fd(callstack, frames, 1);
  write(1, msg3, strlen(msg3));
  _Exit(EXIT_FAILURE);
}

static void SetupSignalHandling(int signum) {
  struct sigaction act;
  memset(&act, 0, sizeof(act));
  act.sa_sigaction = SignalHandler;
  sigaction(signum, &act, 0);
}

void SetupCrashHandling() {
  static bool initialized = false;
  if (!initialized) {
    initialized = true;
    SetupSignalHandling(SIGHUP );       // terminal line hangup
    SetupSignalHandling(SIGINT );       // interrupt program
    SetupSignalHandling(SIGQUIT);       // quit program
    SetupSignalHandling(SIGILL );       // illegal instruction
    SetupSignalHandling(SIGTRAP);       // trace trap
    SetupSignalHandling(SIGABRT);       // abort program (formerly SIGIOT)
    SetupSignalHandling(SIGEMT );       // emulate instruction executed
    SetupSignalHandling(SIGFPE );       // floating-point exception
    SetupSignalHandling(SIGKILL);       // kill program
    SetupSignalHandling(SIGBUS );       // bus error
    SetupSignalHandling(SIGSEGV);       // segmentation violation
    SetupSignalHandling(SIGSYS );       // non-existent system call invoked
    SetupSignalHandling(SIGPIPE);       // write on a pipe with no reader
    SetupSignalHandling(SIGALRM);       // real-time timer expired
    SetupSignalHandling(SIGTERM);       // software termination signal
    SetupSignalHandling(SIGXCPU);       // cpu time limit exceeded
    SetupSignalHandling(SIGXFSZ);       // file size limit exceeded
  }
}

// Setup the signals as early as possible.
struct SetterUpper {
  SetterUpper() { SetupCrashHandling(); }
};
static SetterUpper setter_upper;
