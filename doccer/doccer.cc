
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "doccer_util.h"
#include "doccer_parser.h"

void Usage() {
  fprintf(stderr, "Usage: doccer [-l] [-t template_file] [-Dname=value] "
                  "[-s dictionary_file] [-i image_filename_prefix] "
                  "<filename.doc>\n");
  exit(1);
}

int main(int argc, char **argv) {
  // Process command line arguments.
  const char *template_filename = 0, *filename = 0, *dictionary_filename = 0;
  for (int i = 1; i < argc; i++) {
    if (argv[i][0] == '-') {
      if (argv[i][1] == 't') {
        i++;
        if (i == argc || template_filename) {
          Usage();
        }
        template_filename = argv[i];
      } else if (argv[i][1] == 'l') {
        html = false;
      } else if (argv[i][1] == 'D') {
        char *equals = strchr(argv[i], '=');
        if (!equals) {
          Usage();
        }
        *equals = 0;
        defines[argv[i] + 2] = equals + 1;
      } else if (argv[i][1] == 's') {
        i++;
        if (i == argc || dictionary_filename) {
          Usage();
        }
        dictionary_filename = argv[i];
      } else if (argv[i][1] == 'i') {
        i++;
        if (i == argc || image_filename_prefix) {
          Usage();
        }
        image_filename_prefix = argv[i];
      } else {
        Usage();
      }
    } else {
      if (filename) {
        Usage();
      }
      filename = argv[i];
    }
  }
  if (!template_filename || !filename) {
    Usage();
  }
  if (dictionary_filename) {
    EnableSpellChecking(dictionary_filename);
  }

  // Write the file header by reading the template up until the first '@'.
  FILE *template_file = fopen(template_filename, "rb");
  CHECK(template_file);
  {
    int c = fgetc(template_file);
    while (c != EOF && c != '@') {
      fputc(c, stdout);
      c = fgetc(template_file);
    }
  }

  // Parse the input file. Clip off any leading UTF-8 byte order mark because
  // we don't want to treat that as a word token.
  doccer_lexxer_in = fopen(argv[argc - 1], "rb");
  CHECK(doccer_lexxer_in);
  {
    int c = fgetc(doccer_lexxer_in);
    if (c == 0xef) {
      CHECK(fgetc(doccer_lexxer_in) == 0xbb);
      CHECK(fgetc(doccer_lexxer_in) == 0xbf);
    } else {
      ungetc(c, doccer_lexxer_in);
    }
  }
  line_count = 1;
  doccer_parser_parse();

  // Write the file footer by reading the rest of the template.
  {
    int c = fgetc(template_file);
    while (c != EOF) {
      fputc(c, stdout);
      c = fgetc(template_file);
    }
    fclose(template_file);
  }

  // Spell checking result.
  if (SpellCheckErrors()) {
    Panic("There were spelling errors");
  }

  return 0;
}
