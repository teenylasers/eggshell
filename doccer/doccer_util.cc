
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <map>
#include <string>
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

void ToLowercase(std::string *s) {
  for (int i = 0; i < s->length(); i++) {
    (*s)[i] = tolower((*s)[i]);
  }
}

// The dictionary stores all words in lower case.
static std::map<std::string, bool> dictionary;
static bool spell_checking_enabled = false;
static bool spell_checking_on = true;
static bool spell_check_errors = false;

void EnableSpellChecking(const char *filename) {
  spell_checking_enabled = true;
  FILE *f = fopen(filename, "rb");
  if (!f) {
    Panic("Can't load dictionary from %s", filename);
  }
  char line[1000];
  int count = 0;
  while (fgets(line, sizeof(line), f)) {
    std::string word = line;
    // Remove line ending characters.
    while (word.length() > 0 && word.back() < ' ') {
      word.pop_back();
    }
    ToLowercase(&word);
    dictionary[word] = true;
    count++;
  }
  fclose(f);
  fprintf(stderr, "Loaded %d words to the dictionary\n", count);
}

void CheckSpelling(const char *word) {
  if (!spell_checking_enabled || !spell_checking_on) {
    return;
  }

  // Remove prefix and suffix punctuation and numbers.
  std::string w = word;
  while (w.length() > 0 && !isalpha(w.back())) {
    w.pop_back();
  }
  while (w.length() > 0 && !isalpha(w[0])) {
    w = w.substr(1);
  }

  // Error if it's not in the dictionary.
  ToLowercase(&w);
  if (w.length() > 0 && !dictionary[w]) {
    fprintf(stderr, "Check spelling at line %d: %s\n", line_count, w.c_str());
    spell_check_errors = true;
  }
}

bool SpellCheckErrors() {
  return spell_checking_enabled && spell_check_errors;
}

void AddWordToDictionary(const char *word) {
  std::string w = word;
  ToLowercase(&w);
  dictionary[w] = true;
}

void SpellCheckOff() {
  spell_checking_on = false;
}

void SpellCheckOn() {
  spell_checking_on = true;
}
