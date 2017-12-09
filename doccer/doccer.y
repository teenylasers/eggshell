%{

/*

Doccer parser.

*/

#include <string.h>
#include <stdarg.h>
#include "doccer_util.h"

// Semantic values.
#define YYSTYPE char*

// Error function called by the parser.
int doccer_parser_error(const char *s);

// Section numbering heirarchy
vector<int> section_numbers;
void SectionNumber(int level);
const char *section_label = 0;

// Various kinds of text conversion.
void PrintHTMLAttribute(char *s);
char *ImageFilename(char *filename);

%}

// Tokens.
%token VERBATIM STARTBLOCK ENDBLOCK EMPH BOLD CODE LIST NUMLIST SECTION
       SUBSECTION SUBSUBSECTION TITLE SUBTITLE LINK LINE_BREAK ITEM WORD SPACE
       BLANKLINE INLINE_MATH DISPLAY_MATH NEWCOMMANDS FIGURE EQREF CONTENTS
       AUTHOR TABLE SEPARATOR FORCED_SPACE LABEL ARG

// Misc directives.
%start document
%name-prefix="doccer_parser_"
%error-verbose
%pure-parser

%%

// The whole document.
document: vbox_list ;

// One or more vboxes. Paragraphs are separated by blank lines or other kinds
// of vboxes. The last paragraph need not be followed by anything, unlike
// vbox_list2.
vbox_list:
    paragraph
  | vbox_list2
  | vbox_list2 paragraph
  ;

// One or more vboxes. Paragraphs are separated by blank lines or other kinds
// of vboxes. A blank line or other kind of vbox must follow the last
// paragraph.
vbox_list2:
    vbox
  | vbox_list2 vbox
  ;

// vbox objects are drawn vertically, one above the other, like paragraphs on
// a page.
vbox:
    vbox_no_para
  | paragraph BLANKLINE
  | paragraph vbox_no_para
  | BLANKLINE
  // A paragraph can not start with SPACE. Leading space will be consumed as a
  // vbox and ignored. This scheme ensures that stray spaces don't result in
  // empty paragraphs.
  | SPACE
  ;

// A vbox object not counting a paragraph or blank line.
vbox_no_para:
    LIST { printf(html ? "<ul>\n" : "\\Items{\n"); }
    opt_space_or_blankline
    item_list
    ENDBLOCK { printf(html ? "\n</ul>\n" : "\n}\n"); }
  | NUMLIST { printf(html ? "<ol>\n" : "\\NumItems{\n"); }
    opt_space_or_blankline
    item_list
    ENDBLOCK { printf(html ? "\n</ol>\n" : "\n}\n"); }
  | TITLE   { printf(html ? "<script type='text/javascript'>__the_title__='" : "\\Title{"); } hbox_list ENDBLOCK
            { printf(html ? "'</script><h1 id='title'></h1>\n" : "}\n"); }
  | SUBTITLE { printf(html ? "<h1 class='subtitle'>" : "\\TheSubtitle{"); } hbox_list ENDBLOCK
             { printf(html ? "</h1>\n" : "}\n"); }
  | AUTHOR { printf(html ? "<h1 class='author'>" : "\\TheAuthor{"); } hbox_list ENDBLOCK
           { printf(html ? "</h1>\n" : "}\n"); }
  | CONTENTS ENDBLOCK { printf(html ? "<div class='contents' id='contents'></div>\n"
                                    : "\\Contents\n"); }
  | SECTION opt_label { SectionNumber(0); } hbox_list2 ENDBLOCK { printf(html ? "</h2>\n" : "}\n"); }
  | SUBSECTION opt_label { SectionNumber(1); } hbox_list2 ENDBLOCK { printf(html ? "</h3>\n" : "}\n"); }
  | SUBSUBSECTION opt_label { SectionNumber(2); } hbox_list2 ENDBLOCK { printf(html ? "</h4>\n" : "}\n"); }
  | DISPLAY_MATH { printf(html ? "\\[\\begin{align}%s\\end{align}\\]\n"
                               : "\\begin{align}%s\\end{align}\n", $1); }
  | NEWCOMMANDS { printf(html ? "<p style=\"display:none\">\\(%s\\)</p>\n"
                              : "%s\n", $1); }
  | VERBATIM { printf(html ? "<pre>%s</pre>\n"
                           : "\\begin{Verbatim}[formatcom=\\color{blue}]%s\\end{Verbatim}\n", $1); }
  | FIGURE WORD ENDBLOCK STARTBLOCK
      { printf(html ? "<center><img src='%s' "
                    : "\\Figure{%s}{", ImageFilename($2)); }
    opt_hbox_list ENDBLOCK { printf(html ? "></center>\n" : "}\n") }
  | TABLE { printf(html ? "<table>\n" :
                   "\\begin{longtabu*} to \\textwidth {|X|X|}\n"); }
    opt_space_or_blankline
    table_item_list
    ENDBLOCK { printf(html ? "\n</table>\n" : "\\\\ \\hline \\end{longtabu*}\n"); }
  ;

// An hbox_list that is rendered as a single paragraph.
paragraph: { printf(html ? "<p>" : "\n\n"); } hbox_list
           { printf(html ? "</p>\n" : "\n\n"); } ;

// An optional hbox_list.
opt_hbox_list: | hbox_list ;

// A list of hboxes, SPACE is not allowed at the start.
hbox_list:
    hbox_no_space
  | hbox_no_space hbox_list2
  ;

// A list of hboxes, with SPACE allowed at the start or end.
hbox_list2:
    hbox
  | hbox_list2 hbox
  ;

// hbox object are drawn horizontally, left to right, like words in a
// paragraph.
hbox:
    SPACE { printf(" "); }
  | hbox_no_space
  ;

// An hbox object, not counting space.
hbox_no_space:
    WORD { printf("%s", $1); }
  // | inline_math
  | EQREF { printf("\\eqref{eq:"); } hbox_list ENDBLOCK
          { printf("}"); }
  | EMPH { printf(html ? "<em>" : "\\emph{"); } hbox_list ENDBLOCK
         { printf(html ? "</em>" : "}"); }
  | BOLD { printf(html ? "<b> " : "\\textbf{"); } hbox_list ENDBLOCK
         { printf(html ? "</b>" : "}"); }
  | CODE { printf(html ? "<tt>" : "\\texttt{"); } hbox_list ENDBLOCK
         { printf(html ? "</tt>" : "}"); }
  | LINK { char *link = $1;
           link += 6;                   // Skip the '@link{'
           link[strlen(link) - 1] = 0;  // Remove the }
           if (html) {
             printf("<a href='");
             if (strncmp(link, "http:", 5) != 0 && strncmp(link, "https:", 6) != 0) {
               printf("#");       // Section label in this doc
             }
             PrintHTMLAttribute(link);
             printf("'>");
           } else {
             printf("\\Link{%s}{", link);
           }
         }
    STARTBLOCK hbox_list ENDBLOCK { printf(html ? "</a>" : "}"); }
  | '@' { printf("@"); }
  | '#' { printf(html ? "#" : "\\#"); }
  | '{' { printf(html ? "{" : "\\{"); }
  | '}' { printf(html ? "}" : "\\}"); }
  | FORCED_SPACE { printf(html ? "&nbsp;" : "~"); }
  | LINE_BREAK { printf(html ? "<br>\n" : "\\\n"); }
  | INLINE_MATH { printf(html ? "\\(%s\\)" : "$%s$", $1); }
  | ARG WORD ENDBLOCK  { printf("%s", defines[$2]); }
  ;

// An item list is a vbox list punctuated by ITEM tokens.
item_list:
    item vbox_list
  | item_list item vbox_list
  ;

// Render an item.
item: ITEM { printf(html ? "<li>" : "\\item "); } ;

// A table list is a vbox list punctuated by ITEM tokens.
table_item_list:
    first_table_item vbox_list
  | first_table_item vbox_list rest_of_table_item_list
  ;
rest_of_table_item_list:
    table_item vbox_list
  | rest_of_table_item_list table_item vbox_list
  ;

// Render a table item. The first one needs to be special, for LaTeX.
first_table_item:
    ITEM      { printf(html ? "<tr><td>" : "\\hline "); }
  ;
table_item:
    ITEM      { printf(html ? "<tr><td>" : " \\\\ \\hline "); }
  | SEPARATOR { printf(html ? "<td>" : "&"); }
  ;

opt_space_or_blankline: | SPACE | BLANKLINE ;

opt_label:
                          { section_label = 0; }
  | LABEL WORD ENDBLOCK   { section_label = $2; }
  ;

%%

int doccer_parser_error(const char *s) {
  Panic("%s, at line %d", s, line_count);
}

// Print a section number for the given level (0 = outermost) and advance the
// section counter.

void SectionNumber(int level) {
  if (html) {
    section_numbers.resize(level + 1);
    section_numbers[level]++;
    printf("<h%d class='section_entry' id='", 2 + level);
    if (section_label) {
      printf("%s", section_label);
    } else {
      for (int i = 0; i <= level; i++) {
        printf("%d.", section_numbers[i]);
      }
    }
    printf("'>");
    if (level == 0) {
      printf("<span class='marker'></span>");
    }
    for (int i = 0; i <= level; i++) {
      printf("%d.", section_numbers[i]);
    }
    printf(" ");
  } else {
    printf("\\");
    for (int i = 0; i < level; i++) {
      printf("sub");
    }
    printf("section{");
  }
}

// Print a string in HTML tag attribute context, escaping characters such as '.

void PrintHTMLAttribute(char *s) {
  while (*s) {
    switch (*s) {
      case '\'': puts("%27"); break;
      default: putchar(*s);
    }
    s++;
  }
}

// Do any necessary image filename conversions.

char *ImageFilename(char *filename) {
  if (!html) {
    // In LaTeX, convert *.gif to *-converted.png and rely on a makefile to
    // have generated this file.
    int len = strlen(filename);
    if (len > 4 && strcmp(filename + len - 4, ".gif") == 0) {
      char *f = new char [len + 11];
      strcpy(f, filename);
      strcpy(f + len - 4, "-converted.png");
      return f;
    }
  }
  return filename;
}
