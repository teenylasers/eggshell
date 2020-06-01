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
bool ImageSize(const char *filename, int *width, int *height);

%}

// Tokens.
%token VERBATIM STARTBLOCK ENDBLOCK EMPH BOLD CODE LIST NUMLIST SECTION
       SUBSECTION SUBSUBSECTION TITLE SUBTITLE LINK LINE_BREAK ITEM WORD SPACE
       BLANKLINE INLINE_MATH DISPLAY_MATH NEWCOMMANDS FIGURE EQREF CONTENTS
       AUTHOR TABLE SEPARATOR FORCED_SPACE LABEL ARG HTML DISPLAY_MATH_NONUM
       SPELLING

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
  | DISPLAY_MATH_NONUM { printf(html ? "\\[\\begin{align*}%s\\end{align*}\\]\n"
                                     : "\\begin{align*}%s\\end{align*}\n", $1); }
  | NEWCOMMANDS { printf(html ? "<p style=\"display:none\">\\(%s\\)</p>\n"
                              : "%s\n", $1); }
  | VERBATIM { printf(html ? "<pre>%s</pre>\n"
                           : "\\begin{Verbatim}[formatcom=\\color{blue}]%s\\end{Verbatim}\n", $1); }
  | HTML VERBATIM ENDBLOCK { printf("%s\n", $2); }
  | FIGURE { SpellCheckOff(); } WORD ENDBLOCK STARTBLOCK
      { SpellCheckOn();
        int width, height;
        bool havesize = ImageSize($3, &width, &height);
        if (html) {
          if (havesize) {
            printf("<img class='doc_figure' src='%s%s' width=%d height=%d><div class='doc_caption'>",
                   image_filename_prefix, ImageFilename($3), width/2, height/2);
          } else {
            printf("<img class='doc_figure' src='%s%s'><div class='doc_caption'>",
                   image_filename_prefix, ImageFilename($3));
          }
        } else {
          printf("\\Figure{%s%s}{}", image_filename_prefix, ImageFilename($3));
        }
      }
    opt_hbox_list ENDBLOCK { printf(html ? "</div>\n" : "\n"); }
  | TABLE { printf(html ? "<table class='doc_table'>\n" :
                   "\\begin{longtabu*} to \\textwidth {|X|X|}\n"); }
    opt_space_or_blankline
    table_item_list
    ENDBLOCK { printf(html ? "\n</table>\n" : "\\\\ \\hline \\end{longtabu*}\n"); }
  | SPELLING { SpellCheckOff(); } WORD ENDBLOCK { SpellCheckOn(); AddWordToDictionary($3); }
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
  | EQREF { printf("\\eqref{eq:"); } hbox_list ENDBLOCK
          { printf("}"); }
  | EMPH { printf(html ? "<em>" : "\\emph{"); } hbox_list ENDBLOCK
         { printf(html ? "</em>" : "}"); }
  | BOLD { printf(html ? "<b> " : "\\textbf{"); } hbox_list ENDBLOCK
         { printf(html ? "</b>" : "}"); }
  | CODE { SpellCheckOff(); printf(html ? "<tt>" : "\\texttt{"); } hbox_list ENDBLOCK
         { SpellCheckOn(); printf(html ? "</tt>" : "}"); }
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
  | LABEL { SpellCheckOff(); } WORD ENDBLOCK { SpellCheckOn(); section_label = $3; }
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


class File {
 public:
  explicit File(const char *filename) {
    f_ = fopen(filename, "rb");
    size_ = 0;
    if (f_) {
      fseek(f_, 0, SEEK_END);
      size_ = ftell(f_);
    }
  }

  ~File() { if (f_) fclose(f_); }

  long size() const { return size_; }

  int operator[] (long i) const {
    if (!f_) return 0;
    fseek(f_, i, SEEK_SET);
    return fgetc(f_);
  }

 private:
  FILE *f_;
  long size_;
};

// Determine a PNG width and height. Return true if this seems to be a valid
// PNG, or false otherwise.

bool PngSize(const File &f, int *width, int *height) {
  if (f[0] == 137 && f[1] == 80 && f[2] == 78 && f[3] == 71 && f[4] == 13 &&
      f[5] == 10 && f[6] == 26 && f[7] == 10) {
    *width = (f[16] << 24) | (f[17] << 16) | (f[18] << 8) | f[19];
    *height = (f[20] << 24) | (f[21] << 16) | (f[22] << 8) | f[23];
    return *width >= 0;
  }
  return false;
}

// Determine a JPEG width and height. Return true if this seems to be a valid
// JPEG, or false otherwise.

bool JpegSize(const File &f, int *width, int *height) {
  if (f[0] == 0xff && f[1] == 0xd8) {
    long i = 2;
    while (i < f.size()) {
      int marker = (f[i] << 8) | f[i+1];
      int len = (f[i+2] << 8) | f[i+3];
      if (i == 2) {
        if (!(marker == 0xffe0 /* APP0 */ && f[i+4] == 'J' && f[i+5] == 'F' &&
              f[i+6] == 'I' && f[i+7] == 'F' && f[i+8] == 0)) {
          return false;                           // Not JFIF format
        }
      } else if ((marker & 0xfff0) == 0xffc0) {   // SOFn
        *height = (f[i+5] << 8) | f[i+6];
        *width = (f[i+7] << 8) | f[i+8];
        return true;
      } else if (marker == 0xffda) {              // Start of scan
        return false;
      }
      i += len + 2;
    }
  }
  return false;
}

// Determine a GIF width and height. Return true if this seems to be a valid
// GIF, or false otherwise.

bool GifSize(const File &f, int *width, int *height) {
  if (f[0] == 'G' && f[1] == 'I' && f[2] == 'F' && f[3] == '8' &&
      (f[4] == '7' || f[4] == '9') && f[5] == 'a') {
    *width = (f[7] << 8) | f[6];
    *height = (f[9] << 8) | f[8];
    return true;
  }
  return false;
}

// Determine an image width and height. Return true if this seems to be a valid
// image, or false otherwise.

bool ImageSize(const char *filename, int *width, int *height) {
  File f(filename);
  return PngSize(f, width, height) || JpegSize(f, width, height) ||
         GifSize(f, width, height);
}
