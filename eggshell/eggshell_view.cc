
#include "eggshell_view.h"
#include "shaders.h"
#include "model.h"
#include <QTimer>
#include <QStatusBar>

using Eigen::Vector3d;
using Eigen::Vector3f;
using Eigen::Matrix3d;
using Eigen::Matrix3f;
using Eigen::Matrix4d;
using Eigen::Matrix4f;
using std::vector;

//***************************************************************************
// Utility.

// Render with a bunch of colored directional lights. Use a per-pixel light
// calculation.

static gl::Shader &MultilightShader() {
  static gl::Shader shader(
    #define LIGHT_DIRS \
      " const vec3 light1_dir = vec3(0.7071, 0, 0.7071);" \
      " const vec3 light2_dir = vec3(0.4851 , 0.4851, 0.7276);" \
      " const vec3 light3_dir = vec3(0, 0.7071, 0.7071);" \
      " const vec3 light4_dir = vec3(0, 0, 1);"
    // Vertex shader.
    " #version 330 core\n"
    " uniform mat4 transform, modelview;"
    " uniform mat3 normalmatrix;"
    " uniform float zstretch;"  // If !=0 stretch geometry out along Z
    LIGHT_DIRS
    " in vec3 vertex;"
    " in vec3 normal;"
    " out vec3 normal_es;"      // Interpolated normal in eye space (ES)
    " out vec3 halfway1;"       // Between light and vertex direction (ES)
    " out vec3 halfway2;"
    " out vec3 halfway3;"
    " out vec3 halfway4;"
    " void main() {"
    "   vec3 vertex2 = vertex + vec3(0, 0, sign(vertex.z) * zstretch);"
    "   normal_es = normalmatrix * normal;"
    "   gl_Position = transform * vec4(vertex2, 1.0);"
    "   vec4 p4 = modelview * vec4(vertex2, 1.0);"      // Model point (ES)
    "   vec3 p = vec3(p4) / p4.w;"                      // Model point (ES)
    "   vec3 v = normalize(-p);"                        // View direction (ES)
    "   halfway1 = normalize(light1_dir + v);"
    "   halfway2 = normalize(light2_dir + v);"
    "   halfway3 = normalize(light3_dir + v);"
    "   halfway4 = normalize(light4_dir + v);"
    " }",
    // Fragment shader.
    " #version 330 core\n"
    " const vec3 ambient = vec3(0.4, 0.2, 0.2);"
    LIGHT_DIRS
    " const vec3 light1_col = vec3(0, 0.18, 0.5);"
    " const vec3 light2_col = vec3(0.18, 0.5, 0.18);"
    " const vec3 light3_col = vec3(0.5, 0.18, 0);"
    " const vec3 light4_col = vec3(0, 0, 0.18);"
    " const vec3 specular_color = vec3(1.0, 1.0, 1.0);"
    " in vec3 normal_es;"
    " in vec3 halfway1, halfway2, halfway3, halfway4;"
    " out vec4 fragment_color;"
    " const float shininess = 1000.0;"
    " void main() {"
    "   vec3 n = normalize(normal_es);"           // Normal to surface (ES)
    "   float diffuse1 = max(dot(light1_dir, n), 0.0);"
    "   float diffuse2 = max(dot(light2_dir, n), 0.0);"
    "   float diffuse3 = max(dot(light3_dir, n), 0.0);"
    "   float diffuse4 = max(dot(light4_dir, n), 0.0);"
    "   float specAngle1 = max(dot(normalize(halfway1), n), 0.0);"
    "   float specAngle2 = max(dot(normalize(halfway2), n), 0.0);"
    "   float specAngle3 = max(dot(normalize(halfway3), n), 0.0);"
    "   float specAngle4 = max(dot(normalize(halfway4), n), 0.0);"
    "   float specular1 = pow(specAngle1, shininess);"
    "   float specular2 = pow(specAngle2, shininess);"
    "   float specular3 = pow(specAngle3, shininess);"
    "   float specular4 = pow(specAngle4, shininess);"
    "   vec3 col = ambient +"
    "              diffuse1 * light1_col + "
    "              diffuse2 * light2_col + "
    "              diffuse3 * light3_col + "
    "              diffuse4 * light4_col + "
    "              specular1 * specular_color +"
    "              specular2 * specular_color +"
    "              specular3 * specular_color +"
    "              specular4 * specular_color;"
    "   fragment_color = vec4(col, 0);"
    " }");
    #undef LIGHT_DIRS
  return shader;
}

// Shader to render the ground. A texture is tiled but alternating squares are
// given different colors. This can also be used to render shadows on the
// ground, with objects projected to the ground and drawn with a darkened
// ground texture.

static gl::Shader &GroundShader() {
  static gl::Shader shader(
    // Vertex shader.
    " #version 330 core\n"
    " uniform mat4 transform;"
    " uniform mat4 model_transform;"
    " uniform float zstretch;"  // If !=0 stretch geometry out along Z
    " in vec3 vertex;"
    " out vec2 tex_coord;"
    " void main() {"
    "   vec3 vertex2 = vertex + vec3(0, 0, sign(vertex.z) * zstretch);"
    "   gl_Position = transform * vec4(vertex2, 1.0);"
    "   vec4 model_pos = model_transform * vec4(vertex2, 1.0);"
    "   tex_coord = model_pos.xy;"
    " }",
    // Fragment shader.
    " #version 330 core\n"
    " uniform sampler2D tex;"
    " uniform float brightness;"
    " in vec2 tex_coord;"
    " out vec4 frag_color;"
    " void main() {"
    "   int ix = int(floor(tex_coord.x));"
    "   int iy = int(floor(tex_coord.y));"
    "   float brightness1 = ((ix ^ iy) & 1) * 0.3 + 0.7;"
    "   float brightness2 = (((ix ^ iy) >>  4) & 1) * 0.3 + 0.7;"
    "   vec4 scale = vec4(brightness1, brightness1, brightness2, 0);"
    "   frag_color = scale * brightness * texture(tex, tex_coord);"
    " }");
  return shader;
}

static void RenderCube(bool with_normals) {
  static float data[][6] = {
    // Upper Z face.
    {1, 1, 1, 0,0,1},  {-1, 1, 1, 0,0,1},  {1, -1, 1, 0,0,1},
    {-1, -1, 1, 0,0,1},  {1, -1, 1, 0,0,1},  {-1, 1, 1, 0,0,1},
    // Lower Y face.
    {1, -1, 1, 0,-1,0},  {-1, -1, 1, 0,-1,0},  {1, -1, -1, 0,-1,0},
    {1, -1, -1, 0,-1,0},  {-1, -1, 1, 0,-1,0},  {-1, -1, -1, 0,-1,0},
    // Lower X face.
    {-1, -1, -1, -1,0,0},  {-1, -1, 1, -1,0,0},  {-1, 1, -1, -1,0,0},
    {-1, 1, -1, -1,0,0},  {-1, -1, 1, -1,0,0},  {-1, 1, 1, -1,0,0},
    // Lower Z face.
    {-1, 1, -1, 0,0,-1},  {1, 1, -1, 0,0,-1},  {1, -1, -1, 0,0,-1},
    {-1, 1, -1, 0,0,-1},  {1, -1, -1, 0,0,-1},  {-1, -1, -1, 0,0,-1},
    // Upper Y face.
    {-1, 1, 1, 0,1,0},  {1, 1, 1, 0,1,0},  {1, 1, -1, 0,1,0},
    {-1, 1, 1, 0,1,0},  {1, 1, -1, 0,1,0},  {-1, 1, -1, 0,1,0},
    // Upper X face.
    {1, -1, 1, 1,0,0},  {1, -1, -1, 1,0,0},  {1, 1, -1, 1,0,0},
    {1, -1, 1, 1,0,0},  {1, 1, -1, 1,0,0},  {1, 1, 1, 1,0,0},
  };
  gl::VertexBuffer<float[6]> buffer(3*12, data);
  buffer.Specify1("vertex", 0, 3, GL_FLOAT);
  if (with_normals) {
    buffer.Specify1("normal", 3 * sizeof(float), 3, GL_FLOAT);
  }
  buffer.Draw(GL_TRIANGLES);
}

static void RenderWireframeCube() {
  static float data[2*12][3] = {
    {-1, -1, -1}, { 1, -1, -1}, {-1, -1, -1}, {-1,  1, -1},
    { 1,  1, -1}, {-1,  1, -1}, { 1,  1, -1}, { 1, -1, -1},
    {-1, -1,  1}, { 1, -1,  1}, {-1, -1,  1}, {-1,  1,  1},
    { 1,  1,  1}, {-1,  1,  1}, { 1,  1,  1}, { 1, -1,  1},
    {-1, -1, -1}, {-1, -1,  1}, {-1,  1, -1}, {-1,  1,  1},
    { 1, -1, -1}, { 1, -1,  1}, { 1,  1, -1}, { 1,  1,  1}
  };

  gl::VertexBuffer<float[3]> buffer(2*12, data);
  buffer.Specify1("vertex", 0, 3, GL_FLOAT);
  buffer.Draw(GL_LINES);
}

// This recursively subdivides a triangular area (vertices p1,p2,p3) into
// smaller triangles, and then pushes the position onto the vertex list. All
// triangle vertices are normalized to a distance of 1.0 from the origin
// (p1,p2,p3 are assumed to be already normalized).

static void PushPatch(const Vector3d &p1, const Vector3d &p2,
                      const Vector3d &p3,
                      int level, vector<Vector3f> *list) {
  if (level > 0) {
    Vector3d q1 = (0.5 * (p1 + p2)).normalized();
    Vector3d q2 = (0.5 * (p2 + p3)).normalized();
    Vector3d q3 = (0.5 * (p3 + p1)).normalized();
    PushPatch(p1, q1, q3, level-1, list);
    PushPatch(q1, p2, q2, level-1, list);
    PushPatch(q1, q2, q3, level-1, list);
    PushPatch(q3, q2, p3, level-1, list);
  } else {
    list->push_back(p1.cast<float>());
    list->push_back(p2.cast<float>());
    list->push_back(p3.cast<float>());
  }
}

// This recursively subdivides a line (vertices p1,p2) into smaller lines, and
// then pushes the position onto the vertex list. All vertices are normalized
// to a distance of 1.0 from the origin.

static void PushLine(const Vector3d &p1, const Vector3d &p2,
                     int level, vector<Vector3f> *list) {
  if (level > 0) {
    Vector3d q = (0.5 * (p1 + p2)).normalized();
    PushLine(p1, q, level-1, list);
    PushLine(q, p2, level-1, list);
  } else {
    list->push_back(p1.cast<float>());
    list->push_back(p2.cast<float>());
  }
}

// Icosahedron data for an icosahedron of radius 1.0.
#define ICX 0.525731112119133606f
#define ICZ 0.850650808352039932f
static Vector3d icosahedron_vertex[12] = {
  {-ICX, 0, ICZ}, {ICX, 0, ICZ}, {-ICX, 0, -ICZ}, {ICX, 0, -ICZ},
  {0, ICZ, ICX}, {0, ICZ, -ICX}, {0, -ICZ, ICX}, {0, -ICZ, -ICX},
  {ICZ, ICX, 0}, {-ICZ, ICX, 0}, {ICZ, -ICX, 0}, {-ICZ, -ICX, 0}
};
#undef ICX
#undef ICZ

// Draw a sphere of radius 1.

static void RenderSphere(bool with_normals) {
  static int index[20][3] = {
    {0, 4, 1}, {0, 9, 4}, {9, 5, 4}, {4, 5, 8},
    {4, 8, 1}, {8, 10, 1}, {8, 3, 10}, {5, 3, 8},
    {5, 2, 3}, {2, 7, 3}, {7, 10, 3}, {7, 6, 10},
    {7, 11, 6}, {11, 0, 6}, {0, 1, 6}, {6, 1, 10},
    {9, 0, 11}, {9, 11, 2}, {9, 2, 5}, {7, 2, 11},
  };
  static vector<Vector3f> data;
  static bool data_initialized = false;
  if (!data_initialized) {
    data_initialized = true;
    for (int i = 0; i < 20; i++) {
      PushPatch(icosahedron_vertex[index[i][2]],
                icosahedron_vertex[index[i][1]],
                icosahedron_vertex[index[i][0]],
                3, &data);
    }
  }

  gl::VertexBuffer<Vector3f> buffer(data.size(), data.data());
  buffer.Specify1("vertex", 0, 3, GL_FLOAT);
  if (with_normals) {
    buffer.Specify1("normal", 0, 3, GL_FLOAT);
  }
  buffer.Draw(GL_TRIANGLES);
}

// Draw a wireframe sphere of radius 1.

static void RenderWireframeSphere() {
  static int index[30][2] = {
    {0 , 4 }, {4 , 1 }, {1,  0 }, {0 , 9 }, {9 , 4 }, {9 , 5 }, {5 , 4 },
    {5 , 8 }, {8 , 4 }, {8 , 1 }, {8 , 10}, {10, 1 }, {8 , 3 }, {3 , 10},
    {5 , 3 }, {5 , 2 }, {2 , 3 }, {2 , 7 }, {7 , 3 }, {7 , 10}, {7 , 6 },
    {6 , 10}, {7 , 11}, {11, 6 }, {11, 0 }, {0 , 6 }, {1 , 6 }, {11, 9 },
    {11, 2 }, {2 , 9 }
  };
  static vector<Vector3f> data;
  static bool data_initialized = false;
  if (!data_initialized) {
    data_initialized = true;
    for (int i = 0; i < 30; i++) {
      const Vector3d &p1 = icosahedron_vertex[index[i][0]];
      const Vector3d &p2 = icosahedron_vertex[index[i][1]];
      PushLine(p1, p2, 3, &data);
    }
  }

  gl::VertexBuffer<Vector3f> buffer(data.size(), data.data());
  buffer.Specify1("vertex", 0, 3, GL_FLOAT);
  buffer.Draw(GL_LINES);
}

// Draw a capsule of radius 1 and the given length, with the long axis in the Z
// direction.

static void RenderCapsule(bool with_normals) {
  static vector<Vector3f> data;
  static bool data_initialized = false;
  if (!data_initialized) {
    data_initialized = true;
    const int n = 40;           // Number of longitudial slices (must be even)
    const int m = 10;           // Number of latitudinal slices
    const double offset = 1e-3;
    data.push_back(Vector3f(0, 0, 1));          // North pole
    for (int i = 0; i < n; i++) {
      double sign = (i & 1) ? -1 : 1;
      double lon1 = M_PI * 2 * double(i) / n;
      double lon2 = M_PI * 2 * double((i + 1) % n) / n;
      for (int j = 1; j <= m; j++) {
        double lat = M_PI * 0.5 * double(j) / m;
        double z = sign*std::max(cos(lat), offset);
        data.push_back(Vector3f(cos(lon1)*sin(lat), sin(lon1)*sin(lat), z));
        data.push_back(Vector3f(cos(lon2)*sin(lat), sin(lon2)*sin(lat), z));
      }
      for (int j = m; j >= 1; j--) {
        double lat = M_PI * 0.5 * double(j) / m;
        double z = -sign*std::max(cos(lat), offset);
        data.push_back(Vector3f(cos(lon1)*sin(lat), sin(lon1)*sin(lat), z));
        data.push_back(Vector3f(cos(lon2)*sin(lat), sin(lon2)*sin(lat), z));
      }
      data.push_back(Vector3f(0, 0, -sign));
    }
  }

  gl::VertexBuffer<Vector3f> buffer(data.size(), data.data());
  buffer.Specify1("vertex", 0, 3, GL_FLOAT);
  if (with_normals) {
    buffer.Specify1("normal", 0, 3, GL_FLOAT);
  }
  buffer.Draw(GL_TRIANGLE_STRIP);
}

// Draw a wireframe capsule of radius 1 and the given length, with the long
// axis in the Z direction.

static void RenderWireframeCapsule() {
  static vector<Vector3f> data;
  static bool data_initialized = false;
  if (!data_initialized) {
    data_initialized = true;
    const int n = 8;            // Number of longitudial slices (must be even)
    const int m = 10;           // Number of latitudinal slices
    const double offset = 1e-3;
    data.push_back(Vector3f(0, 0, 1));          // North pole
    for (int i = 0; i < n; i++) {
      double sign = (i & 1) ? -1 : 1;
      double lon = M_PI * 2 * double(i) / n;
      for (int j = 1; j <= m; j++) {
        double lat = M_PI * 0.5 * double(j) / m;
        double z = sign*std::max(cos(lat), offset);
        data.push_back(Vector3f(cos(lon)*sin(lat), sin(lon)*sin(lat), z));
      }
      for (int j = m; j >= 1; j--) {
        double lat = M_PI * 0.5 * double(j) / m;
        double z = -sign*std::max(cos(lat), offset);
        data.push_back(Vector3f(cos(lon)*sin(lat), sin(lon)*sin(lat), z));
      }
      data.push_back(Vector3f(0, 0, -sign));
    }
  }

  gl::VertexBuffer<Vector3f> buffer(data.size(), data.data());
  buffer.Specify1("vertex", 0, 3, GL_FLOAT);
  buffer.Draw(GL_LINE_STRIP);
}

//***************************************************************************
// The functions in model.h that are called by SimulationStep().

struct Object {
  enum Type { SPHERE, BOX, CAPSULE, POINT, LINE };
  Type type;
  Vector3d center, halfside;
  Matrix3d R;
};

vector<Object> objects;

void DrawSphere(const Vector3d &center, const Matrix3d &rotation,
                double radius) {
  objects.resize(objects.size() + 1);
  objects.back().type = Object::SPHERE;
  objects.back().center = center;
  objects.back().halfside = Vector3d(radius, radius, radius);
  objects.back().R = rotation;
}

void DrawBox(const Vector3d &center, const Matrix3d &rotation,
             const Vector3d &side_lengths) {
  objects.resize(objects.size() + 1);
  objects.back().type = Object::BOX;
  objects.back().center = center;
  objects.back().halfside = side_lengths / 2;
  objects.back().R = rotation;
}

void DrawCapsule(const Vector3d &center, const Matrix3d &rotation,
                 double radius, double length) {
  objects.resize(objects.size() + 1);
  objects.back().type = Object::CAPSULE;
  objects.back().center = center;
  objects.back().halfside = Vector3d(radius, radius, length / 2 + radius);
  objects.back().R = rotation;
}

void DrawPoint(const Eigen::Vector3d &position) {
  objects.resize(objects.size() + 1);
  objects.back().type = Object::POINT;
  objects.back().center = position;
  objects.back().halfside.setZero();
  objects.back().R.setIdentity();
}

void DrawLine(const Eigen::Vector3d &pos1, const Eigen::Vector3d &pos2) {
  objects.resize(objects.size() + 1);
  objects.back().type = Object::LINE;
  objects.back().center = (pos1 + pos2) * 0.5;
  objects.back().halfside = (pos2 - pos1) * 0.5;
  objects.back().R.setIdentity();
}

inline void SetModelTransform(const Matrix4d &T) {
  GLuint loc = GL(GetUniformLocation)(gl::CurrentProgram(), "model_transform");
  if (loc != -1) {
    Matrix4f Tf = T.cast<float>();
    GL(UniformMatrix4fv)(loc, 1, GL_FALSE, Tf.data());
  }
}

inline void SetZStretch(double stretch) {
  GLuint loc = GL(GetUniformLocation)(gl::CurrentProgram(), "zstretch");
  if (loc != -1) {
    GL(Uniform1f)(loc, stretch);
  }
}

static void DrawObjects(bool with_normals, bool wireframe,
                        const Matrix4d &projection = Matrix4d::Identity()) {
  Matrix4d P = gl::Projection();
  Matrix4d M = gl::ModelView();

  for (int i = 0; i < objects.size(); i++) {
    Matrix4d T;
    T.setZero();
    T(3, 3) = 1;
    T.block(0, 3, 3, 1) = objects[i].center;
    if (objects[i].type == Object::SPHERE) {
      T.block(0, 0, 3, 3) = objects[i].R * objects[i].halfside[0];
      gl::ApplyTransform(P, M * projection * T);
      SetModelTransform(projection * T);
      if (wireframe) {
        RenderWireframeSphere();
      } else {
        RenderSphere(with_normals);
      }
    } else if (objects[i].type == Object::BOX) {
      T.block(0, 0, 3, 3) = objects[i].R;
      T *= gl::Scale(objects[i].halfside);
      gl::ApplyTransform(P, M * projection * T);
      SetModelTransform(projection * T);
      if (wireframe) {
        RenderWireframeCube();
      } else {
        RenderCube(with_normals);
      }
    } else if (objects[i].type == Object::CAPSULE) {
      T.block(0, 0, 3, 3) = objects[i].R;
      double scale = objects[i].halfside[0];
      T *= gl::Scale(scale);
      gl::ApplyTransform(P, M * projection * T);
      SetModelTransform(projection * T);
      SetZStretch(objects[i].halfside[2] / scale - 1);
      if (wireframe) {
        RenderWireframeCapsule();
      } else {
        RenderCapsule(with_normals);
      }
      SetZStretch(0);
    }
  }
}

//***************************************************************************
// EggshellView.

EggshellView::EggshellView(QWidget *parent) : GLViewer(parent) {
  running_ = false;
  show_bounding_box_ = false;
  status_bar_ = 0;
  ground_texture_ = 0;

  // Take the first step of the simulation so that we have objects to draw.
  objects.clear();
  SimulationStep();

  // Setup camera.
  Look(GLViewer::LOOK_AT_YZ_PLANE_FROM_MINUS_X);
  GetCamera().pos[2] = 2;
}

EggshellView::~EggshellView() {
  delete ground_texture_;
}

void EggshellView::Link(QStatusBar *status_bar) {
  status_bar_ = status_bar;
}

void EggshellView::ToggleRunning() {
  running_ ^= 1;
  if (running_) {
    QTimer::singleShot(0.001, this, &EggshellView::OnSimulationTimeout);
  }
  update();
}

void EggshellView::ToggleShowBoundingBox() {
  show_bounding_box_ ^= 1;
  update();
}

void EggshellView::OnSimulationTimeout() {
  if (running_) {
    objects.clear();
    SimulationStep();
    update();
    QTimer::singleShot(0.001, this, &EggshellView::OnSimulationTimeout);
  }
}

void EggshellView::Draw() {
  GL(Enable)(GL_DEPTH_TEST);
  GL(Enable)(GL_CULL_FACE);
  GL(DepthFunc)(GL_LESS);
  GL(DepthRange)(0, 0.99999);   // Needed for glEnable(GL_DEPTH_CLAMP) to work
  GL(Enable)(GL_MULTISAMPLE);
  GL(CullFace)(GL_BACK);
  GL(FrontFace)(GL_CCW);

  // Prevent pixels outside the near/far view planes from being clipped. This
  // is needed to draw the ground out to the horizon.
  GL(Enable)(GL_DEPTH_CLAMP);

  // Clear the buffer.
  GL(ClearColor)(0.2, 0.5, 0.8, 0);
  GL(Clear)(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Draw the ground. The depth test is disabled for this because we draw
  // shadows on top of the ground.
  {
    GL(DepthFunc)(GL_ALWAYS);
    GroundShader().Use();
    gl::SetUniform("brightness", 1);
    SetModelTransform(Matrix4d::Identity());
    ApplyCameraTransformations();
    if (!ground_texture_) {
      QImage img("ground.jpg", "JPG");
      CHECK(!img.isNull());
      QImage img2 = img.convertToFormat(QImage::Format_RGB888);
      const uint8_t *bits = img2.bits();
      ground_texture_ = new gl::Texture2D(
            img.width(), img.height(), const_cast<uint8_t*>(bits));
    }
    ground_texture_->Bind();
    GL(TexParameteri)(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    GL(TexParameteri)(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                      GL_LINEAR_MIPMAP_LINEAR);

    float xy[][2] = {{-100, -100}, {100, -100}, {-100, 100}, {100, 100}};
    gl::VertexBuffer<float[2]> buffer(4, xy);
    buffer.Specify1("vertex", 0, 2, GL_FLOAT);
    buffer.Draw(GL_TRIANGLE_STRIP);

    // Draw ground shadows, if the viewpoint is above the ground.
    if (GetCamera().pos[2] > 0) {
      Matrix4d project_shadow;
      project_shadow.setZero();
      project_shadow.setIdentity();
      project_shadow(0, 2) = 1;   // X direction of the shadow
      project_shadow(1, 2) = 1;   // Y direction of the shadow
      project_shadow(2, 2) = 0;
      GL(Disable)(GL_CULL_FACE);
      gl::SetUniform("brightness", 0.6);
      DrawObjects(false, false, project_shadow);
      GL(Enable)(GL_CULL_FACE);
    }
    GL(DepthFunc)(GL_LESS);
  }

  // Set the shader and lights.
  MultilightShader().Use();
  ApplyCameraTransformations();

  // Draw the model.
  DrawObjects(true, false);

  // Draw wireframe things and points.
  static gl::Shader wireframe_shader(
    // Vertex shader.
    " #version 330 core\n"
    " uniform mat4 transform;"
    " uniform float zstretch;"  // If !=0 stretch geometry out along Z
    " in vec3 vertex;"
    " void main() {"
    "   vec3 vertex2 = vertex + vec3(0, 0, sign(vertex.z) * zstretch);"
    "   gl_Position = transform * vec4(vertex2, 1.0);"
    " }",
    // Fragment shader.
    " #version 330 core\n"
    " uniform vec3 color;"
    " out vec4 fragment_color;"
    " void main() {"
    "   fragment_color = vec4(color, 0.0);"
        // Adjust the fragment Z values so we don't get Z fighting with the
        // already-rendered polygons.
    "   gl_FragDepth = gl_FragCoord.z - 0.0005;"
    " }");
  wireframe_shader.Use();

  // Optionally show the bounding box.
  if (show_bounding_box_) {
    ApplyCameraTransformations();
    gl::DrawThick(2, 2, false, [&]() {
      double b[6];
      GetBoundingBox(b);
      gl::SetUniform("color", 1, 1, 0);
      gl::DrawCubeWireframe(
            Vector3d((b[0]+b[1])*0.5, (b[2]+b[3])*0.5, (b[4]+b[5])*0.5),
            Vector3d(b[1]-b[0], b[3]-b[2], b[5]-b[4]));
    });
  }

  // Draw wireframes around the objects.
  ApplyCameraTransformations();
  gl::SetUniform("color", 0, 0, 0);
  gl::DrawThick(2, 2, false, [&]() {
    DrawObjects(true, true);
  });

  // Draw lines from the objects list.
  {
    ApplyCameraTransformations();
    GL(Disable)(GL_DEPTH_TEST);
    gl::SetUniform("color", 1, 1, 0);
    vector<Vector3f> p;
    for (int i = 0; i < objects.size(); i++) {
      if (objects[i].type == Object::LINE) {
        p.push_back((objects[i].center - objects[i].halfside).cast<float>());
        p.push_back((objects[i].center + objects[i].halfside).cast<float>());
      }
    }
    gl::VertexBuffer<Vector3f> buffer(p);
    buffer.Specify1("vertex", 0, 3, GL_FLOAT);
    gl::DrawThick(2, 2, false, [&]() {
      buffer.Draw(GL_LINES);
    });
    GL(Enable)(GL_DEPTH_TEST);
  }

  // Draw points from the objects list.
  {
    ApplyCameraTransformations();
    GL(Disable)(GL_DEPTH_TEST);
    gl::SetUniform("color", 1, 1, 0);
    vector<Vector3f> p;
    for (int i = 0; i < objects.size(); i++) {
      if (objects[i].type == Object::POINT) {
        p.push_back(objects[i].center.cast<float>());
      }
    }
    gl::VertexBuffer<Vector3f> buffer(p);
    buffer.Specify1("vertex", 0, 3, GL_FLOAT);
    GL(PointSize)(10);
    buffer.Draw(GL_POINTS);
    GL(Enable)(GL_DEPTH_TEST);
  }
}

void EggshellView::GetBoundingBox(double bounds[6]) {
  for (int i = 0; i < 3; i++) {
    bounds[i*2 + 0] = __DBL_MAX__;
    bounds[i*2 + 1] = -__DBL_MAX__;
  }
  // Compute combined bounding box for all objects.
  for (int i = 0; i < objects.size(); i++) {
    const Vector3d &pos = objects[i].center;
    const Matrix3d &R = objects[i].R;
    const Vector3d &halfside = objects[i].halfside;
    for (int x = -1; x <= 1; x += 2) {
      for (int y = -1; y <= 1; y += 2) {
        for (int z = -1; z <= 1; z += 2) {
          Vector3d p = pos + R * Vector3d(halfside[0]*x, halfside[1]*y,
                                          halfside[2]*z);
          for (int j = 0; j < 3; j++) {
            bounds[2*j  ] = std::min(bounds[2*j  ], p[j]);
            bounds[2*j+1] = std::max(bounds[2*j+1], p[j]);
          }
        }
      }
    }
  }
}
