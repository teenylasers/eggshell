
#ifndef __CLIMBING_VIEW_H__
#define __CLIMBING_VIEW_H__

#include "viewer.h"
#include "gl_utils.h"

class QStatusBar;

class EggshellView : public GLViewer {
 public:
  EggshellView(QWidget *parent);
  ~EggshellView();

  void Link(QStatusBar *status_bar);
  void ToggleRunning();
  void SingleStep();
  void ToggleShowBoundingBox();
  void OnSimulationTimeout();

 private:
  bool running_, show_bounding_box_, single_step_;

  // View state.
  QStatusBar *status_bar_;
  gl::Texture2D *ground_texture_;

  // Virtual functions from GLViewer
  void Draw() override;
  void GetBoundingBox(double bounds[6]) override;
};

#endif
