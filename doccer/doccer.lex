%{

/*

Doccer lexxer.

Any amount of whitespace (and/or comments) between any non-whitespace entities
will be treated as a single SPACE or BLANKLINE token.

Allowing SPACE and BLANKLINE tokens (rather than simply hiding whitespace
from the parser) allows us to distinguish "foo@bar" from "foo @bar" from
"foo@ bar" from "foo @ bar", etc.

*/

// Semantic values.
#define YYSTYPE char*

#include <string>
#include "doccer_util.h"
#include "doccer_parser.h"

// Complain about a bad character in the input.
static void BadChar(int c) {
  if (c >= 32 && c <= 126) {
    Panic("Unexpected character '%c', at line %d", c, line_count);
  } else {
    Panic("Unexpected character #%d, at line %d", (unsigned char) c,
          line_count);
  }
}

// Forward and external declarations.
int doccer_parser_error(const char *s);
static bool SkipCommentAndWhitespace(char first_char);
static void ReadVerbatim(const char *prefix, char **ret_string);
static void ReadTeXMath(char **ret_string);
static char *Escape(const char *s, bool math_context = false);

// Debugging:
//#define TOKEN(tok) { printf("\nTOKEN: %s\n", #tok); return tok; }
#define TOKEN(tok) { return tok; }

%}

%option prefix="doccer_lexxer_"
%option noyywrap

%%

@=*"["                                  { ReadVerbatim(yytext, yylval); TOKEN(VERBATIM); }
[ #\t\n]                                { if (SkipCommentAndWhitespace(yytext[0])) TOKEN(BLANKLINE) else TOKEN(SPACE); }
"{"                                     { TOKEN(STARTBLOCK); }
"}"                                     { TOKEN(ENDBLOCK); }
"@m{"                                   { ReadTeXMath(yylval); TOKEN(INLINE_MATH); }
"@M{"                                   { ReadTeXMath(yylval); TOKEN(DISPLAY_MATH); }
"@eqref{"                               { TOKEN(EQREF); }
"@emph{"                                { TOKEN(EMPH); }
"@b{"                                   { TOKEN(BOLD); }
"@c{"                                   { TOKEN(CODE); }
"@list{"                                { TOKEN(LIST); }
"@numlist{"                             { TOKEN(NUMLIST); }
"@section{"                             { TOKEN(SECTION); }
"@subsection{"                          { TOKEN(SUBSECTION); }
"@subsubsection{"                       { TOKEN(SUBSUBSECTION); }
"@label{"                               { TOKEN(LABEL); }
"@subtitle{"                            { TOKEN(SUBTITLE); }
"@author{"                              { TOKEN(AUTHOR); }
"@title{"                               { TOKEN(TITLE); }
"@contents{"                            { TOKEN(CONTENTS); }
"@link{"[^}]*"}"                        { *yylval = strdup(yytext); TOKEN(LINK); }
"@figure{"                              { TOKEN(FIGURE); }
"@newcommands{"                         { ReadTeXMath(yylval); TOKEN(NEWCOMMANDS); }
"@table{"                               { TOKEN(TABLE); }
"@arg{"                                 { TOKEN(ARG); }
"@_"                                    { TOKEN(LINE_BREAK); }
"@*"                                    { TOKEN(ITEM); }
"@|"                                    { TOKEN(SEPARATOR); }
"@~"                                    { TOKEN(FORCED_SPACE); }
"@"                                     { doccer_parser_error("Unknown command"); }

"@@"                                    { TOKEN('@'); }
"@#"                                    { TOKEN('#'); }
"@{"                                    { TOKEN('{'); }
"@}"                                    { TOKEN('}'); }
([!-\377]{-}[@#{}])+                    { *yylval = Escape(yytext); TOKEN(WORD); }

"\r"                                    { doccer_parser_error("CR characters found at end of line"); }
.                                       { if (yytext[0]) BadChar(yytext[0]); }

%%

// These functions are defined down here because they need to call yyinput() or
// other lexxer functions.

// Return the number of characters that match 'c' in the given string.

static int CountChars(const char *s, char c) {
  int count = 0;
  while (*s) {
    if (*s == c) {
      count++;
    }
    s++;
  }
  return count;
}

// Skip through comments and whitespace in the file. Return true if the
// BLANKLINE token should be returned (if more than two \n's were encountered)
// or false if the SPACE token shoud be returned. The logic here allows a
// blank line to be bracketable by comments, e.g.
//
//     Words on a page.
//     # Comment.          }
//                         }--- turns into single BLANKLINE token
//     # Comment.          }
//     Words on a page.
//
// Also multiple blank line tokens should not be generatable thusly:
//     # Comment.          }
//                         }
//     # Comment.          }--- turns into single BLANKLINE token
//                         }
//     # Comment.          }

static bool SkipCommentAndWhitespace(char first_char) {
  bool in_comment = (first_char == '#');
  bool blank_line_found = false;
  int newline_count = 0;
  if (first_char == '\n') {
    line_count++;
    newline_count = 1;
  }

  for (;;) {
    int c = yyinput();
    if (c == '\n') {
      line_count++;
    } else if (c == EOF) {
      return blank_line_found;
    }

    if (in_comment) {
      // Process comment.
      if (c == '\n') {
        in_comment = false;
        newline_count = 1;
      }
    } else {
      // Process whitespace.
      if (c == '#') {
        in_comment = true;
      } else if (c == '\n') {
        newline_count++;
        if (newline_count >= 2) {
          blank_line_found = true;
        }
      } else if (c == ' ' || c == '\t') {
        // Skip whitespace character.
      } else {
        unput(c);
        return blank_line_found;
      }
    }
  }
}

// This scans the input until it finds ]===@ where the number of '=' characters
// to match is the number present in the prefix. If ret_string is nonzero then
// it is set to a string containing the prefix plus the scanned characters.

static void ReadVerbatim(const char *prefix, char **ret_string) {
  int equals_to_match = CountChars(prefix, '=');
  int startline = line_count;
  int state = 0;
  std::string buffer;
  for (;;) {
    int c = yyinput();
    buffer.push_back(c);
    if (c == EOF) {
      line_count = startline;
      doccer_parser_error("Unclosed verbatim block");
    } else if (c == '\n') {
      line_count++;
    }

    if (state == equals_to_match + 1) {
      if (c == '@') {
        CHECK(buffer.length() >= equals_to_match + 2);
        buffer[buffer.length() - equals_to_match - 2] = 0;
        *ret_string = strdup(buffer.c_str());
        return;
      }
      state = 0;
    } else if (state == 0) {
      if (c == ']') {
        state = 1;
      }
    } else {
      if (c == '=') {
        state++;
      }
      else if (c == ']') {
        state = 1;
      }
      else {
        state = 0;
      }
    }
  }
}

// Given that we've just scanned a '@M{' or '@m{' sequence, process characters
// until we reach the end of a TeX math block. We match curly braces and
// ignored escapes like \{. Blank lines are erased, as LaTeX doesn't like them
// within display math blocks.

static void ReadTeXMath(char **ret_string) {
  std::string buffer;
  int startline = line_count;
  int depth = 1;
  int last_c = 0;
  bool ignore_next_char = false;
  for (;;) {
    int c = yyinput();
    if (c != '\n' || last_c != '\n') {
      // Don't store more than one consecutive \n.
      buffer.push_back(c);
    }
    if (c == EOF) {
      line_count = startline;
      doccer_parser_error("Unclosed math block");
    } else if (c == '\n') {
      line_count++;
    }

    if (ignore_next_char) {
      ignore_next_char = false;
    } else {
      if (c == '{') {
        depth++;
      } else if (c == '}') {
        depth--;
        if (depth == 0) {
          buffer.resize(buffer.size() - 1);     // Chop off final '}'
          *ret_string = Escape(buffer.c_str(), true);
          return;
        }
      } else if (c == '\\') {
        ignore_next_char = true;
      }
    }
    last_c = c;
  }
}

// Like strdup() but transforms some character sequences appropriately
// depending on the mode. For HTML:
//   <  --> &lt;
//   >  --> &gt;
//   &  --> &amp;
//   `` --> &ldquo;
//   '' --> &rdquo;
// For LaTeX:
//   &  --> \&
//   %  --> \%
//   $  --> \$
//   _  --> \_
//   ~  --> \textasciitilde
//   ^  --> \textasciicircum
//   \  --> \textbackslash
//   Note that the LaTeX characters #,{,} are doccer commands that are escaped
//   by @#, @{, @} and so they are not specially escaped here.

static char *Escape(const char *s, bool math_context) {
  std::string buffer;
  if (html) {
    // HTML mode.
    while (*s) {
      if (*s == '<') {
        buffer.append("&lt;");
      } else if (*s == '>') {
        buffer.append("&gt;");
      } else if (*s == '&') {
        buffer.append("&amp;");
      } else if (*s == '`' && s[1] == '`') {
        buffer.append("&ldquo;");
        s++;
      } else if (*s == '\'' && s[1] == '\'') {
        buffer.append("&rdquo;");
        s++;
      } else {
        buffer.push_back(*s);
      }
      s++;
    }
  } else {
    // LaTeX mode.
    if (math_context) {
      // In math context the user is writing plain LaTeX, nothing needs to be
      // escaped.
      return strdup(s);
    }
    while (*s) {
      if (*s == '&') {
        buffer.append("\\& ");
      } else if (*s == '%') {
        buffer.append("\\% ");
      } else if (*s == '$') {
        buffer.append("\\$ ");
      } else if (*s == '#') {
        buffer.append("\\# ");
      } else if (*s == '_') {
        buffer.append("\\_ ");
      } else if (*s == '{') {
        buffer.append("\\{ ");
      } else if (*s == '}') {
        buffer.append("\\} ");
      } else if (*s == '~')  {
        buffer.append("\\textasciitilde ");
      } else if (*s == '^')  {
        buffer.append("\\textasciicircum ");
      } else if (*s == '\\') {
        buffer.append("\\textbackslash ");
      } else {
        buffer.push_back(*s);
      }
      s++;
    }
  }
  return strdup(buffer.c_str());
}
