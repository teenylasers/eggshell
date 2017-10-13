
#include <stdio.h>

int main() {
  printf("char user_script_util_dot_lua[] = {\n  ");
  for (int count = 1; true; count++) {
    int c = fgetc(stdin);
    if (c == EOF) {
      printf("0};\nint user_script_util_dot_lua_length = %d;\n",
             count - 1);
      return 0;
    }
    printf("0x%02x, ", c);
    if ((count % 8) == 0) {
      printf("\n  ");
    }
  }
}
