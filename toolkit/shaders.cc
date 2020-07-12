// Copyright (C) 2014-2020 Russell Smith.
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

#include "shaders.h"
#include "error.h"
#include "myvector"

using std::vector;

namespace gl {

GLuint CreateProgramFromShaders(const char *vertex_shader,
                                const char *fragment_shader) {
  GLuint vshader = GL(CreateShader)(GL_VERTEX_SHADER);
  GLuint fshader = GL(CreateShader)(GL_FRAGMENT_SHADER);
  CHECK(vshader && fshader);
  GLint result = GL_FALSE;
  int log_length;

  // Compile and check vertex shader.
  GL(ShaderSource)(vshader, 1, &vertex_shader, NULL);
  GL(CompileShader)(vshader);
  GL(GetShaderiv)(vshader, GL_COMPILE_STATUS, &result);
  GL(GetShaderiv)(vshader, GL_INFO_LOG_LENGTH, &log_length);
  if (log_length > 0) {
    vector<char> msg(log_length + 1);
    GL(GetShaderInfoLog)(vshader, log_length, NULL, &msg[0]);
    Panic("GLSL vertex shader: %s", &msg[0]);
  }

  // Compile and check fragment shader.
  GL(ShaderSource)(fshader, 1, &fragment_shader, NULL);
  GL(CompileShader)(fshader);
  GL(GetShaderiv)(fshader, GL_COMPILE_STATUS, &result);
  GL(GetShaderiv)(fshader, GL_INFO_LOG_LENGTH, &log_length);
  if (log_length > 0) {
    vector<char> msg(log_length + 1);
    GL(GetShaderInfoLog)(fshader, log_length, NULL, &msg[0]);
    Panic("GLSL fragment shader: %s", &msg[0]);
  }

  // Link and check the program.
  GLuint program = GL(CreateProgram)();
  GL(AttachShader)(program, vshader);
  GL(AttachShader)(program, fshader);
  GL(LinkProgram)(program);
  GL(GetProgramiv)(program, GL_LINK_STATUS, &result);
  GL(GetProgramiv)(program, GL_INFO_LOG_LENGTH, &log_length);
  if (log_length > 0) {
    vector<char> msg(log_length + 1);
    GL(GetProgramInfoLog)(program, log_length, NULL, &msg[0]);
    Panic("GLSL program: %s", &msg[0]);
  }

  GL(DeleteShader)(vshader);
  GL(DeleteShader)(fshader);
  return program;
}

Shader &FlatShader() {
  static Shader shader(
    // Vertex shader.
    " #version 330 core\n"
    " uniform mat4 transform;"
    " in vec3 vertex;"
    " void main() {"
    "   gl_Position = transform * vec4(vertex, 1.0);"
    " }",
    // Fragment shader.
    " #version 330 core\n"
    " uniform vec3 color;"
    " out vec4 fragment_color;"
    " void main() {"
    "   fragment_color = vec4(color, 0.0);"
    " }");
  return shader;
}

Shader &SmoothShader() {
  static Shader shader(
    // Vertex shader.
    " #version 330 core\n"
    " uniform mat4 transform;"
    " in vec3 vertex;"
    " in vec4 vertex_color;"
    " out vec4 color;"
    " void main() {"
    "   gl_Position = transform * vec4(vertex, 1.0);"
    "   color = vertex_color;"
    " }",
    // Fragment shader.
    " #version 330 core\n"
    " in vec4 color;"
    " out vec4 fragment_color;"
    " void main() {"
    "   fragment_color = color;"
    " }");
  return shader;
}

Shader &PerPixelLightingShader() {
  static Shader shader(
    // Light direction in eye space (unit length vector).
    #define LIGHTDIR " const vec3 light_dir = vec3(0.5345, 0.2673, 0.8018); "
    // Vertex shader.
    " #version 330 core\n"
    " uniform mat4 transform, modelview;"
    " uniform mat3 normalmatrix;"
    " in vec3 vertex;"
    " in vec3 normal;"
    " out vec3 normal_es;"      // Interpolated normal in eye space (ES)
    " out vec3 halfway;"        // Between light and vertex direction (ES)
    LIGHTDIR
    " void main() {"
    "   normal_es = normalize(normalmatrix * normal);"
    "   gl_Position = transform * vec4(vertex, 1.0);"
    "   vec4 p4 = modelview * vec4(vertex, 1.0);"       // Model point (ES)
    "   vec3 p = vec3(p4) / p4.w;"                      // Model point (ES)
    "   vec3 v = normalize(-p);"                        // View direction (ES)
    "   halfway = normalize(light_dir + v);"
    " }",
    // Fragment shader.
    " #version 330 core\n"
    " uniform vec3 color;"
    " in vec3 normal_es;"
    " in vec3 halfway;"
    " out vec4 fragment_color;"
    LIGHTDIR
    " const vec4 specular_color = vec4(1.0, 1.0, 1.0, 0.0);"
    " const float shininess = 100.0;"
    " const float ambient = 0.5;"
    " void main() {"
    "   vec3 n = normalize(normal_es);"           // Normal to surface (ES)
    "   float diffuse = max(dot(light_dir, n), 0.0);"
    "   float specAngle = max(dot(normalize(halfway), n), 0.0);"
    "   float specular = pow(specAngle, shininess);"
    "   fragment_color = (ambient + diffuse) * vec4(color, 0.0) + "
    "                    specular * specular_color;"
    " }");
    #undef LIGHTDIR
  return shader;
}

}  // namespace gl
