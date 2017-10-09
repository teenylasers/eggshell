# Toolkit

Various standalone utility things. Each set of files with a common prefix are
intended to go together and to be used by multiple applications. Here are the
standards that apply across all files:

* Conform (mostly) to Google C++ style guide, use of C++11 okay.
* Use a separate class (or namespace if there are many symbols) for each thing.
* `error.h` defines the shared error handling mechanism. It is not okay to use
  `CHECK()` or `Panic()` for recoverable errors, instead error codes should be
  returned to the caller (with optional output of human readable errors through
  `Error()`).
* Use include guard names of the form `__TOOLKIT_<filename>__`
* Features such as availability of libraries can be configured via
  `__TOOLKIT_xxx__` macros defined on the command line.
