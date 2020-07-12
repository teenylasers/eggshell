// Rama Simulator, Copyright (C) 2014-2020 Russell Smith.
//
// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.

#include <stdio.h>

int main(int argc, char **argv) {
  if (argc != 2) {
    return 1;
  }
  printf("char %s[] = {\n  ", argv[1]);
  for (int count = 1; true; count++) {
    int c = fgetc(stdin);
    if (c == EOF) {
      printf("0};\nint %s_length = %d;\n", argv[1], count - 1);
      return 0;
    }
    printf("0x%02x, ", c);
    if ((count % 8) == 0) {
      printf("\n  ");
    }
  }
  return 0;
}
