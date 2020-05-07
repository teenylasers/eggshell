
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
