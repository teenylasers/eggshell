
#ifndef __DOCCER_UTIL_H__
#define __DOCCER_UTIL_H__

#include <stdio.h>
#include <vector>
#include <map>
#include <string>

using std::vector;
using std::map;
using std::string;

#define CHECK(cond) if(!(cond)) \
  { Panic("Check failed at %s:%d : " #cond, __FILE__, __LINE__); }

// The mode we're in.
extern bool html;       // True if HTML, false if LaTeX

// Global variables and functions for the parser and lexxer.
extern FILE *doccer_lexxer_in;
extern int line_count;
extern int doccer_parser_parse();
extern int doccer_lexxer_lex(char **yylval_param);
extern int doccer_parser_lex(char **yylval_param);
extern map<string, const char *> defines;

// Error handling.
void Panic(const char *msg, ...)
  __attribute__((format (printf, 1, 2), noreturn));

// Enable spell checking with the given dictionary file.
void EnableSpellChecking(const char *filename);

// Spell check a word. Print an error if it is not in the dictionary.
void CheckSpelling(const char *word);

// Return true if there were any spell check errors.
bool SpellCheckErrors();

// Add a word to the spell checking dictionary.
void AddWordToDictionary(const char *word);

// Start and stop spell checking.
void SpellCheckOff();
void SpellCheckOn();

#endif
