
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include "doccer_util.h"

int line_count = 1;
bool html = true;
map<string, const char *> defines;

void Panic(const char *msg, ...) {
  va_list ap;
  va_start(ap, msg);
  fflush(stdout);
  fflush(stderr);
  fprintf(stderr, "Error: ");
  vfprintf(stderr, msg, ap);
  fprintf(stderr, "\n");
  fflush(stderr);
  exit(1);
}

int doccer_parser_lex(char **yylval_param) {
  return doccer_lexxer_lex(yylval_param);
}
