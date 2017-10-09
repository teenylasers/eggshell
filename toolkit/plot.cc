
#include "plot.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <algorithm>
#include <map>
#include <cmath>
#include "error.h"
#include "testing.h"

using std::vector;
using std::string;
using std::isfinite;

namespace Plot {

// Nonconfigurable plot geometry and other settings.
const int kMaxLabels = 12;              // Max number of labels on each axis
const double kTickLength = 5;           // Tick length in pixels
const double kMinTickSpacing = 35;      // Min spacing between ticks / labels
const double kMinPlotWidth = 10;        // Min size of plot window (pixels)
const double kMinPlotHeight = 10;       // Min size of plot window (pixels)
const char *kLabelFormat = "%.10g";     // Used to sprintf the labels
const double kLegendBorder = 5;         // Border around legend
const double kLegendInset = 10;         // Gap between legend and plot border
const double kLegendLen = 20;           // Line sample length in legend

// Fraction of label font height to raise super text
const double kSuper = 0.5;

// Gap between super and sub text is this fraction of small label font height.
const double kSupSubGap = 0.0;

// Minimum size of a valid selection rectangle (pixels).
const double kMinSelectionRectangle = 3;

//***************************************************************************
// Utility.

// Convert an index to {1,2,5}*10^N. 0,1,2,3... maps to 1,2,5,10..., etc.
static double IndexTo125eN(int index) {
  const int kOffset = 0x40000002;       // Must be divisible by 3
  double base = pow(10, (index + kOffset) / 3 - kOffset / 3);
  switch ((index + kOffset) % 3) {
    case 0: return base;
    case 1: return base * 2;
    case 2: return base * 5;
  }
  return 0;
}

// Round x up to the next {1,2,5}*10^N and return the 125eN index. This assumes
// x > 0.
static int RoundUpTo125eN(double x) {
  const double kEpsilon = 1e-9;
  double base_index = floor(log10(x));
  double base = pow(10, base_index);          // 10^N <= x < 10^(N+1)
  if (x <= base * (1 + kEpsilon)) return base_index * 3;
  if (x <= base * (2 + kEpsilon)) return base_index * 3 + 1;
  if (x <= base * (5 + kEpsilon)) return base_index * 3 + 2;
  return base_index * 3 + 3;
}

// Return two candidates for the most common value of floor(log10(N)) for all
// numbers N in the range lo..hi. Return in 'N' the candidates and in 'range'
// the corresponding spans of values in lo..hi that have this value of
// floor(log10()). Assume lo >= 0 and hi > lo.
static void MajorityFloorLog10Helper(double lo, double hi,
                                     double N[2], double range[2]) {
  N[1] = floor(log10(hi));
  N[0] = N[1] - 1;
  double q = pow(10, N[1]);
  if (lo >= q) {
    // All values lo..hi have the same floor(log10()).
    range[0] = 0;
    range[1] = hi - lo;
  } else {
    double k = std::max(lo, pow(10, N[0]));
    range[0] = q - k;
    range[1] = hi - q;
  }
}

// Return the most common value of floor(log10(|N|)) for all numbers N in the
// range lo..hi. Assume only that hi > lo.
static double MajorityFloorLog10(double lo, double hi) {
  double N[4], range[4];
  if (lo >= 0) {
    // All lo..hi positive.
    MajorityFloorLog10Helper(lo, hi, N, range);
    return (range[0] > range[1]) ? N[0] : N[1];
  } else if (hi <= 0) {
    // All lo..hi negative.
    MajorityFloorLog10Helper(-hi, -lo, N, range);
    return (range[0] > range[1]) ? N[0] : N[1];
  } else {
    // We have lo < 0 and hi > 0, so we compute separate result for positive
    // and negative numbers and then combine the results.
    double smaller = std::min(-lo, hi);    // Smaller in magnitude
    double larger  = std::max(-lo, hi);    // Larger in magnitude
    MajorityFloorLog10Helper(0, smaller, N,     range);
    MajorityFloorLog10Helper(0, larger , N + 2, range + 2);
    if (N[0] == N[2]) {
      // Completely overlapping N values, so add ranges:  [0] [1]
      //                                                  [2] [3]
      range[2] += range[0];
      range[3] += range[1];
    } else if (N[1] == N[2]) {
      // Partially overlapping N values, add ranges:  [0] [1]
      //                                                  [2] [3]
      range[2] += range[1];
    } else {
      // Nonoverlapping ranges:  [0] [1]
      //                                  [2] [3]
    }
    return (range[2] > range[3]) ? N[2] : N[3];
  }
}

// Chop the given string into pieces that differ by their superscript/subscript
// level. The sequences ^{...} and _{...} are used to denote superscript and
// subscript text respectively.

static void ParseSuperSubText(const char *s,
                              vector<std::pair<int, string> > *pieces) {
  pieces->clear();
  vector<int> stack(1);                 // Stack of levels, stacking with 0
  char last_char = 0;                   // Char scanned on last loop iteration
  while (true) {
    char emit = 0;                      // Character to emit, if not 0
    int emit_level = stack.back();      // Level to emit character at
    int next_last_char = 0;
    if (last_char == '\\') {
      // Ignore the '\', emit this chararacter.
      emit = *s;
    } else if (last_char == '^' && *s == '{') {
      // Start a new superscript level.
      stack.push_back(stack.back() + 1);
    } else if (last_char == '_' && *s == '{') {
      // Start a new subscript level.
      stack.push_back(stack.back() - 1);
    } else {
      emit = last_char;                 // Emit last character if it's not 0
      if (*s == '}') {
        if (stack.size() > 1) {
          stack.pop_back();             // Pop the level stack
        }
      } else {
        next_last_char = *s;            // Regular character, save it
      }
    }
    last_char = next_last_char;

    // Emit a character if necessary.
    if (emit) {
      if (pieces->empty() || pieces->back().first != emit_level) {
        pieces->resize(pieces->size() + 1);
        pieces->back().first = emit_level;
      }
      pieces->back().second += emit;
    }

    if (!*s) {
      break;
    }
    s++;
  }
}

//***************************************************************************
// Window.

Window::~Window() {
}

double Window::FormattedTextWidth(const char *s) {
  Pieces pieces;
  ParseSuperSubText(s, &pieces);
  return FormattedTextWidthHelper(pieces);
}

double Window::FormattedTextHeight(const char *s, double *descent) {
  Pieces pieces;
  ParseSuperSubText(s, &pieces);
  return FormattedTextHeightHelper(pieces, descent);
}

void Window::DrawFormattedText(const char *s, double x, double y,
                               TextAlignment halign, TextAlignment valign) {
  // Measure the label fonts.
  UseLabelFont();
  double descent0;
  double h0 = TextHeight(&descent0);
  UseLabelSmallFont();
  double h1 = TextHeight();

  // We do alignment so that we retain the baseline of the regular (non-super
  // or subscript) text.
  Pieces pieces;
  ParseSuperSubText(s, &pieces);
  double descent;
  double width = FormattedTextWidthHelper(pieces);
  double height = FormattedTextHeightHelper(pieces, &descent);
  ::AlignText(width, height, descent, halign, valign, false, &x, &y);
  y = y - height + descent - descent0 + 1;

  for (int i = 0; i < pieces.size(); i++) {
    if (pieces[i].first == 0) {
      UseLabelFont();
      DrawText(pieces[i].second.c_str(), x, y,
               TEXT_ALIGN_LEFT, TEXT_ALIGN_BOTTOM);
    } else if (pieces[i].first > 0) {
      UseLabelSmallFont();
      DrawText(pieces[i].second.c_str(), x, y + round(kSuper * h0),
               TEXT_ALIGN_LEFT, TEXT_ALIGN_BOTTOM);
    } else if (pieces[i].first < 0) {
      UseLabelSmallFont();
      DrawText(pieces[i].second.c_str(),
               x, y + round(kSuper * h0 - (kSupSubGap + 1) * h1),
               TEXT_ALIGN_LEFT, TEXT_ALIGN_BOTTOM);
    }
    x += TextWidth(pieces[i].second.c_str());
  }
}

double Window::FormattedTextWidthHelper(const Pieces &pieces) {
  double width = 0;
  for (int i = 0; i < pieces.size(); i++) {
    if (pieces[i].first == 0) {
      UseLabelFont();
    } else {
      UseLabelSmallFont();
    }
    width += TextWidth(pieces[i].second.c_str());
  }
  return width;
}

double Window::FormattedTextHeightHelper(const Pieces &pieces,
                                         double *descent) {
  UseLabelFont();
  double ydescent;
  double h0 = TextHeight(&ydescent);
  UseLabelSmallFont();
  double h1 = TextHeight();
  double ymin = 0, ymax = h0 - 1;

  for (int i = 0; i < pieces.size(); i++) {
    if (pieces[i].first > 0) {
      ymax = std::max(h0, round(kSuper * h0) + h1 - 1);
    } else if (pieces[i].first < 0) {
      ymin = std::min(0.0, round(kSuper * h0 - (kSupSubGap + 1) * h1));
    }
  }
  if (descent) {
    *descent = ydescent - ymin;
  }
  return ymax - ymin + 1;
}

//***************************************************************************
// Automatic axis layout. This is the actual hard part of plotting a graph.

AxisLayoutInput::AxisLayoutInput() {
  memset(this, 0, sizeof(*this));
}

AxisLayoutOutput::AxisLayoutOutput() {
  // The default will cause no tick marks or labels to be drawn, which is why
  // lmin > lmax.
  log10_scale = 0;
  axis_min = 0;
  axis_max = 0;
  tick = 0;
  lmin = 1;
  lmax = 0;
}

void AxisLayout(Window *win,
                const AxisLayoutInput &in, AxisLayoutOutput *out) {
  // When the plot area is too small to support ticks that satisfy the various
  // spacing constraints we have only bad options. We could show just one tick
  // mark and its label, which will render nicely but will not actually
  // describe the axis properly. Or, we could show multiple ticks and labels
  // but risk them overlapping and looking messy. We go with the latter option,
  // mainly because it is consistent with what Matlab does.

  const double kOnePlus = 1.0 + 1e-9;   // A little more than one
  const double kOneMinus = 1.0 - 1e-9;  // A little less than one

  // The number of ticks in the current (i.e. 'out') solution, or -1 if 'out'
  // is not yet assigned.
  int out_ticks = -1;

  // Handle the special case where the input data spans zero range, as
  // otherwise the tick computations below are not meaningful.
  double data_min = in.data_min;
  double data_max = in.data_max;
  if (data_max - data_min == 0) {
    if (data_min == 0) {
      data_min = -1;
      data_max = 1;
    } else {
      data_max *= 2;
      data_min = 0;
    }
  }

  // Get a lower bound starting point for 'tick', the space between axis
  // ticks/labels in graph coordinates (assuming a tight axis bound).
  int tick_index_lo = RoundUpTo125eN((data_max - data_min) / kMaxLabels);
  for (int tick_index = tick_index_lo; tick_index <= tick_index_lo + 6;
       tick_index++) {
    AxisLayoutOutput trial;
    trial.tick = IndexTo125eN(tick_index);

    // Compute the axis bounds given the value of 'tick'.
    if (in.tight_fit) {
      trial.axis_min = data_min;
      trial.axis_max = data_max;
      trial.lmin = ceil (data_min / trial.tick);
      trial.lmax = floor(data_max / trial.tick);
    } else {
      // lmin..lmax will span at least 2 ticks:
      trial.lmin = floor((data_min * kOnePlus) / trial.tick);
      trial.lmax = ceil ((data_max * kOneMinus) / trial.tick);
      trial.axis_min = trial.lmin * trial.tick;
      trial.axis_max = trial.lmax * trial.tick;
    }

    // If there will be too many labels displayed then try the next largest
    // tick value.
    int num_ticks = trial.lmax - trial.lmin + 1;
    if (num_ticks > kMaxLabels) {
      continue;
    }

    // If less than two ticks are displayed then larger tick values can not be
    // useful (e.g. one tick by itself does not describe the axis properly), so
    // go with the current best result.
    if (num_ticks < 2) {
      break;
    }

    // Compute the axis label scale, i.e. divide all axis labels by this before
    // display.
    trial.log10_scale = MajorityFloorLog10(trial.axis_min * kOnePlus,
                                           trial.axis_max * kOneMinus);
    if (trial.log10_scale > -3 && trial.log10_scale < 4) {
      trial.log10_scale = 0;
    }
    double scale = pow(10, trial.log10_scale);

    // Compute the number of pixels per tick.
    double tick_px = trial.tick / (trial.axis_max - trial.axis_min) *
                     in.size_px;

    // Compute the labels and the minimum amount of space between them.
    vector<string> labels;
    double last_label_max_px = 0;
    bool labels_too_close_together = false;
    for (int64_t i = trial.lmin; i <= trial.lmax; i++) {
      char s[100];
      snprintf(s, sizeof(s), kLabelFormat, (i * trial.tick) / scale);
      labels.push_back(s);
      double size_px = in.x_axis ? win->TextWidth(s) : win->TextHeight();
      double offset_px = (i * trial.tick - trial.axis_min)
          / (trial.axis_max - trial.axis_min) * in.size_px;
      double label_min_px = offset_px - size_px / 2;
      double label_max_px = offset_px + size_px / 2;
      if (i > trial.lmin) {
        double gap_px = label_min_px - last_label_max_px;
        if (gap_px < in.min_label_gap_px) {
          labels_too_close_together = true;
        }
      }
      last_label_max_px = label_max_px;
    }

    // If tick_px is too small or the labels are too close together then this
    // trial may not be the best available, but we will keep it as a fallback
    // in case nothing else works (but only if it is a new solution or has a
    // different number of ticks from the previous best).
    if (out_ticks == -1 || num_ticks != out_ticks) {
      *out = trial;
      out->labels.swap(labels);
      out_ticks = num_ticks;
    }
    if (tick_px < kMinTickSpacing || labels_too_close_together) {
      continue;
    }

    // If we get to this point then we found a configuration that meets all the
    // constraints.
    break;
  }

  // We should have assigned 'out' at this point. But just in case there's a
  // bug and we didn't, make sure 'out' is in a consistent state and no ticks
  // are drawn.
  if (out_ticks == -1) {
    out->log10_scale = 0;
    out->axis_min = in.data_min;
    out->axis_max = in.data_max;
    out->tick = 1;
    out->lmin = 1;
    out->lmax = 0;
    out->labels.clear();
  }
}

//***************************************************************************
// Plot2D.

Plot2D::Plot2D(Window *win) {
  win_ = win;
  Clear();
}

void Plot2D::Clear() {
  plot_x1_ = 0;
  plot_y1_ = 0;
  x_axis_label_.clear();
  y_axis_label_.clear();
  title_.clear();
  grid_ = false;
  first_x_ = first_y_ = 0;
  last_x_ = last_y_ = 0;
  last_button_state_ = 0;
  traces_.clear();

  axis_.clear();
  axis_.resize(1);
  axis_[0].in[0].x_axis = true;
  axis_[0].in[0].data_min = __DBL_MAX__;
  axis_[0].in[0].data_max = -__DBL_MAX__;
  axis_[0].in[0].size_px = 0;
  axis_[0].in[0].tight_fit = false;

  axis_[0].in[1].x_axis = false;
  axis_[0].in[1].data_min = __DBL_MAX__;
  axis_[0].in[1].data_max = -__DBL_MAX__;
  axis_[0].in[1].size_px = 0;
  axis_[0].in[1].tight_fit = false;
}

Plot2D::~Plot2D() {
  win_->DestroyImage(image_handle_);
}

void Plot2D::AddTrace(int n, const double *x, const double *y,
                      uint32_t rrggbb, double line_width) {
  if (!UpdateDataBoundsFromTrace(n, x, y)) {
    // No valid points in x,y so nothing to do.
    return;
  }

  // Select color automatically if necessary.
  if (rrggbb == -1) {
    const int kNumColors = 7;
    static uint32_t colors[kNumColors] = {
      0x0000ff,         // Blue
      0xff0000,         // Red
      0x00c000,         // Green
      0x00c0ff,         // Cyan (or more of a sky blue)
      0xffc000,         // Darkish yellow
      0xff00ff,         // Magenta
      0x000000,         // Black
    };
    rrggbb = colors[traces_.size() % kNumColors];
  }

  // Add trace if non-empty.
  if (n > 0) {
    traces_.resize(traces_.size() + 1);
    traces_.back().x.resize(n);
    traces_.back().y.resize(n);
    memcpy(traces_.back().x.data(), x, n * sizeof(double));
    memcpy(traces_.back().y.data(), y, n * sizeof(double));
    traces_.back().color = rrggbb;
    traces_.back().line_width = line_width;
  }

  // Mark non-finite points so that it's easier to test for them.
  for (int i = 0; i < n; i++) {
    if (!isfinite(x[i]) || !isfinite(y[i])) {
      traces_.back().x[i] = INFINITY;
      traces_.back().y[i] = INFINITY;
    }
  }
}

void Plot2D::AddTraceLabel(const char *label) {
  traces_.back().label = label;
}

void Plot2D::AddImage(int width, int height, uint8_t *rgb_data) {
  win_->DestroyImage(image_handle_);
  image_handle_ = win_->CreateImage(width, height, rgb_data);
  double x[2] = {0, double(width)};
  double y[2] = {0, double(height)};
  UpdateDataBoundsFromTrace(2, x, y);
}

void Plot2D::Draw() {
  // Update geometry from the current window.
  win_->UseLabelFont();
  plot_x1_ = round(win_->TextWidth("XXX-0.0000"));
  plot_y1_ = round(win_->TextHeight() * 2.5 + kTickLength * 3);
  axis_[0].in[0].min_label_gap_px = win_->TextWidth(" ");
  axis_[0].in[1].min_label_gap_px = win_->TextHeight();
  double win_width = win_->Width();
  double win_height = win_->Height();

  // Recompute axis layout for the current set of traces at the current zoom
  // level.
  {
    double w = round(std::max(kMinPlotWidth, win_width - plot_x1_ * 1.5));
    double h = round(std::max(kMinPlotHeight, win_height - 2 * plot_y1_));
    for (int i = 0; i < axis_.size(); i++) {
      axis_[i].in[0].size_px = w;
      axis_[i].in[1].size_px = h;
    }
  }
  if (traces_.empty() && image_handle_.pointer == 0) {
    // Use default AxisLayoutOutput which will cause no tick marks or labels to
    // be drawn.
  } else {
    for (int i = 0; i < 2; i++) {
      AxisLayout(win_, axis_.back().in[i], &axis_.back().out[i]);
    }
  }

  // Get plot area.
  const double plot_x2 = plot_x1_ + axis_.back().in[0].size_px;
  const double plot_y2 = plot_y1_ + axis_.back().in[1].size_px;
  const AxisLayoutOutput &a0 = axis_.back().out[0];
  const AxisLayoutOutput &a1 = axis_.back().out[1];

  // Clear the graph area to the background color.
  win_->Color(0xffffff);
  win_->Rectangle(plot_x1_, plot_y1_, plot_x2 - plot_x1_, plot_y2 - plot_y1_);

  // Draw grid lines.
  if (grid_) {
    win_->Color(0x808080);
    for (int64_t i = a0.lmin; i <= a0.lmax; i++) {
      double x = GraphToWinX(i * a0.tick);
      win_->StartLine(Window::GRID);
      win_->Line(x, plot_y1_, x, plot_y2);
      win_->EndLine();
    }
    for (int64_t i = a1.lmin; i <= a1.lmax; i++) {
      double y = GraphToWinY(i * a1.tick);
      win_->StartLine(Window::GRID);
      win_->Line(plot_x1_, y, plot_x2, y);
      win_->EndLine();
    }
  }

  // Draw the image, if any.
  if (image_handle_.pointer) {
    // @@@ Might want to fine tune exact pixel alignment with respect to border
    // and axis.
    win_->DrawImage(image_handle_, GraphToWinX(0),
                    GraphToWinY(image_handle_.height),
                    GraphToWinX(image_handle_.width) - GraphToWinX(0),
                    GraphToWinY(image_handle_.height) - GraphToWinY(0));
  }

  // Draw traces. When we are very zoomed in we don't want to pass extremely
  // large coordinate values to win_->Line(), as many window systems will
  // misbehave when the coordinate values get too large. Thus we do our own
  // clipping here and pass only coordinate values that are in the window.
  // Infinite or NaN data points will break the trace.
  for (int i = 0; i < traces_.size(); i++) {
    const Trace &t = traces_[i];
    if (t.x.size() < 2) {
      continue;
    }
    win_->Color(t.color);
    win_->LineWidth(t.line_width);
    win_->StartLine(Window::TRACE);
    for (int j = 1; j < t.x.size(); j++) {
      if (t.x[j - 1] != INFINITY && t.x[j] != INFINITY) {
        double x1 = GraphToWinX(t.x[j - 1]);
        double y1 = GraphToWinY(t.y[j - 1]);
        double x2 = GraphToWinX(t.x[j]);
        double y2 = GraphToWinY(t.y[j]);
        if (ClipLine(win_width, win_height, &x1, &y1, &x2, &y2)) {
          win_->Line(x1, y1, x2, y2);
        }
      }
    }
    win_->EndLine();
  }
  win_->LineWidth(1);

  // Draw X and Y axis tick marks.
  win_->Color(0);
  for (int64_t i = a0.lmin; i <= a0.lmax; i++) {
    double x = GraphToWinX(i * a0.tick);
    win_->StartLine(Window::BORDER);
    win_->Line(x, plot_y1_, x, plot_y1_ + kTickLength);
    win_->EndLine();
    win_->StartLine(Window::BORDER);
    win_->Line(x, plot_y2, x, plot_y2 - kTickLength);
    win_->EndLine();
  }
  for (int64_t i = a1.lmin; i <= a1.lmax; i++) {
    double y = GraphToWinY(i * a1.tick);
    win_->StartLine(Window::BORDER);
    win_->Line(plot_x1_, y, plot_x1_ + kTickLength, y);
    win_->EndLine();
    win_->StartLine(Window::BORDER);
    win_->Line(plot_x2, y, plot_x2 - kTickLength, y);
    win_->EndLine();
  }

  // Clear the graph border to the background color (with 4 rectangles)
  win_->Color(0xcccccc);
  win_->Rectangle(0, 0, win_width, plot_y1_);
  win_->Rectangle(0, plot_y2, win_width, win_height - plot_y2);
  win_->Rectangle(0, plot_y1_, plot_x1_, plot_y2 - plot_y1_);
  win_->Rectangle(plot_x2, plot_y1_, win_width - plot_x2, plot_y2 - plot_y1_);

  // Draw X and Y axis tick labels.
  win_->UseLabelFont();
  win_->Color(0);
  for (int64_t i = a0.lmin; i <= a0.lmax; i++) {
    double x = GraphToWinX(i * a0.tick);
    double y = plot_y1_ - kTickLength;
    win_->DrawText(a0.labels[i - a0.lmin].c_str(), x, y,
                   TEXT_ALIGN_CENTER, TEXT_ALIGN_TOP);
  }
  for (int64_t i = a1.lmin; i <= a1.lmax; i++) {
    double x = plot_x1_ - kTickLength;
    double y = GraphToWinY(i * a1.tick);
    win_->DrawText(a1.labels[i - a1.lmin].c_str(), x, y,
                   TEXT_ALIGN_RIGHT, TEXT_ALIGN_CENTER);
  }

  // Draw X axis scale label.
  if (a0.log10_scale != 0) {
    win_->UseLabelFont();
    double label_height = win_->TextHeight();
    char s[100];
    snprintf(s, sizeof(s), "x 10^{%.0f}", a0.log10_scale);
    win_->DrawFormattedText(s,
                            plot_x2, plot_y1_ - 2 * kTickLength - label_height,
                            TEXT_ALIGN_RIGHT, TEXT_ALIGN_TOP);
  }

  // Draw Y axis scale label.
  if (a1.log10_scale != 0) {
    char s[100];
    snprintf(s, sizeof(s), "x 10^{%.0f}", a1.log10_scale);
    win_->DrawFormattedText(s, plot_x1_, plot_y2 + kTickLength,
                            TEXT_ALIGN_LEFT, TEXT_ALIGN_BOTTOM);
  }

  // Draw axis labels and the title.
  win_->Color(0);
  win_->UseLabelFont();
  win_->DrawText(x_axis_label_.c_str(), (plot_x1_ + plot_x2) / 2, kTickLength,
                 TEXT_ALIGN_CENTER, TEXT_ALIGN_BOTTOM);
  win_->DrawText(y_axis_label_.c_str(), kTickLength, (plot_y1_ + plot_y2) / 2,
                 TEXT_ALIGN_CENTER, TEXT_ALIGN_TOP, true);
  win_->DrawText(title_.c_str(), win_width / 2, win_height - kTickLength,
                 TEXT_ALIGN_CENTER, TEXT_ALIGN_TOP);

  // Draw plot area border.
  win_->Color(0);
  win_->LineWidth(1);
  win_->StartLine(Window::BORDER);
  win_->Line(plot_x1_, plot_y1_, plot_x1_, plot_y2 );
  win_->Line(plot_x1_, plot_y2 , plot_x2,  plot_y2 );
  win_->Line(plot_x2,  plot_y2 , plot_x2,  plot_y1_);
  win_->Line(plot_x2,  plot_y1_, plot_x1_, plot_y1_);
  win_->Line(plot_x1_, plot_y1_, plot_x1_, plot_y1_);
  win_->EndLine();

  // Compute the legend size.
  double legend_w = 0, legend_h = 0;
  for (int i = 0; i < traces_.size(); i++) {
    if (!traces_[i].label.empty()) {
      legend_w = std::max(legend_w,
                          win_->FormattedTextWidth(traces_[i].label.c_str()));
      legend_h += win_->FormattedTextHeight(traces_[i].label.c_str());
    }
  }

  // Draw the legend.
  if (legend_w > 0 && legend_h > 0) {
    double x_ofs = plot_x1_ + kLegendInset;
    double y_ofs = plot_y1_ + kLegendInset;
    legend_w += 3 * kLegendBorder + kLegendLen;
    legend_h += 2 * kLegendBorder;
    win_->Color(0xffffff);
    win_->Rectangle(x_ofs, y_ofs, legend_w, legend_h);
    win_->Color(0);
    win_->StartLine(Window::BORDER);
    win_->Line(x_ofs, y_ofs, x_ofs, y_ofs + legend_h );
    win_->Line(x_ofs, y_ofs + legend_h , x_ofs + legend_w,  y_ofs + legend_h );
    win_->Line(x_ofs + legend_w,  y_ofs + legend_h , x_ofs + legend_w,  y_ofs);
    win_->Line(x_ofs + legend_w,  y_ofs, x_ofs, y_ofs);
    win_->Line(x_ofs, y_ofs, x_ofs, y_ofs);
    win_->EndLine();
    int y = y_ofs + legend_h - kLegendBorder;
    for (int i = 0; i < traces_.size(); i++) {
      if (!traces_[i].label.empty()) {
        const Trace &t = traces_[i];
        win_->Color(0);
        win_->DrawFormattedText(t.label.c_str(),
                                x_ofs + 2*kLegendBorder + kLegendLen, y,
                                TEXT_ALIGN_LEFT, TEXT_ALIGN_TOP);
        double h = win_->FormattedTextHeight(t.label.c_str());
        win_->Color(t.color);
        win_->LineWidth(t.line_width);
        win_->StartLine(Window::TRACE);
        win_->Line(x_ofs + kLegendBorder, y - h/2,
                   x_ofs + kLegendBorder + kLegendLen, y - h/2);
        win_->EndLine();
        y -= h;
      }
    }
  }
}

void Plot2D::SetXAxis(double x1, double x2) {
  axis_[0].in[0].data_min = std::min(x1, x2);
  axis_[0].in[0].data_max = std::max(x1, x2);
}

void Plot2D::SetYAxis(double y1, double y2) {
  axis_[0].in[1].data_min = std::min(y1, y2);
  axis_[0].in[1].data_max = std::max(y1, y2);
}

void Plot2D::EventMouse(double x, double y, int button_state) {
  // Don't do anything if there are no traces.
  if (traces_.empty() && image_handle_.pointer == 0) {
    return;
  }

  // Erase the old selection rectangle if necessary.
  if ((last_button_state_ & 1) == 0) {
    first_x_ = x;
    first_y_ = y;
  } else {
    if (fabs(first_x_ - last_x_) >= kMinSelectionRectangle &&
        fabs(first_y_ - last_y_) >= kMinSelectionRectangle) {
      win_->SelectionRectangle(first_x_, first_y_, last_x_, last_y_, false);
    }
  }

  // The right mouse button pops the zoom stack.
  if ((last_button_state_ & 4) == 0 && (button_state & 4)) {
    if (axis_.size() > 1) {
      axis_.pop_back();
      win_->Refresh();
    }
  }

  // Save state.
  last_button_state_ = button_state;
  last_x_ = x;
  last_y_ = y;

  // Draw the new selection rectangle if necessary.
  if (fabs(first_x_ - x) >= kMinSelectionRectangle &&
      fabs(first_y_ - y) >= kMinSelectionRectangle) {
    if (button_state & 1) {
      win_->SelectionRectangle(first_x_, first_y_, last_x_, last_y_, true);
    } else {
      // A selection rectangle has been defined, push a new set of axes onto
      // the zoom stack. If we zoom in too far then we will run out of bits of
      // precision and we will have fx==lx or fy==ly. In that case refuse to
      // zoom in.
      double fx = WinXToGraph(first_x_);
      double fy = WinYToGraph(first_y_);
      double lx = WinXToGraph(last_x_);
      double ly = WinYToGraph(last_y_);
      if (fx == lx || fy == ly) {
        // Refuse to zoom in.
      } else {
        axis_.push_back(axis_.back());
        axis_.back().in[0].data_min = std::min(fx, lx);
        axis_.back().in[0].data_max = std::max(fx, lx);
        axis_.back().in[0].tight_fit = true;
        axis_.back().in[1].data_min = std::min(fy, ly);
        axis_.back().in[1].data_max = std::max(fy, ly);
        axis_.back().in[1].tight_fit = true;
      }
      win_->Refresh();
    }
  }
}

double Plot2D::GraphToWinX(double x) const {
  const AxisLayoutOutput &a0 = axis_.back().out[0];
  return (x - a0.axis_min) / (a0.axis_max - a0.axis_min) *
         axis_.back().in[0].size_px + plot_x1_;
}

double Plot2D::GraphToWinY(double y) const {
  const AxisLayoutOutput &a1 = axis_.back().out[1];
  return (y - a1.axis_min) / (a1.axis_max - a1.axis_min) *
         axis_.back().in[1].size_px + plot_y1_;
}

double Plot2D::WinXToGraph(double x) const {
  const AxisLayoutOutput &a0 = axis_.back().out[0];
  return a0.axis_min + (x - plot_x1_) * (a0.axis_max - a0.axis_min) /
                                        axis_.back().in[0].size_px;
}

double Plot2D::WinYToGraph(double y) const {
  const AxisLayoutOutput &a1 = axis_.back().out[1];
  return a1.axis_min + (y - plot_y1_) * (a1.axis_max - a1.axis_min) /
                                        axis_.back().in[1].size_px;
}

bool Plot2D::UpdateDataBoundsFromTrace(int n,
                                       const double *x, const double *y) {
  bool found_valid_point = false;
  for (int i = 0; i < n; i++) {
    if (isfinite(x[i]) && isfinite(y[i])) {
      axis_[0].in[0].data_min = std::min(axis_[0].in[0].data_min, x[i]);
      axis_[0].in[0].data_max = std::max(axis_[0].in[0].data_max, x[i]);
      axis_[0].in[1].data_min = std::min(axis_[0].in[1].data_min, y[i]);
      axis_[0].in[1].data_max = std::max(axis_[0].in[1].data_max, y[i]);
      found_valid_point = true;
    }
  }
  return found_valid_point;
}

bool Plot2D::ClipLine(double w, double h,
                      double *_x1, double *_y1, double *_x2, double *_y2) {
  #define SWAP(a, b) { double tmp = (a); (a) = (b); (b) = tmp; }
  double x1 = *_x1;
  double y1 = *_y1;
  double x2 = *_x2;
  double y2 = *_y2;
  int polarity = 0;     // Counts the number of endpoint swaps

  // Force y1 <= y2.
  if (y1 > y2) {
    SWAP(x1, x2)
    SWAP(y1, y2)
    polarity ^= 1;
  }

  // Clip at y = 0 and y = h.
  if (y2 < 0 || y1 > h) {
    return false;
  }
  if (y1 < 0 || y2 > h) {
    // Above tests guarantee that y1 != y2 and that the line intersects y=0 or
    // y=h or both.
    double dx = x2 - x1;
    double dy = y2 - y1;
    double alpha1 = std::max(0.0,    - y1  / dy);
    double alpha2 = std::min(1.0, (h - y1) / dy);
    x2 = x1 + dx * alpha2;
    y2 = y1 + dy * alpha2;
    x1 += dx * alpha1;
    y1 += dy * alpha1;
  }

  // Force x1 <= x2.
  if (x1 > x2) {
    SWAP(x1, x2)
    SWAP(y1, y2)
    polarity ^= 1;
  }

  // Clip at x = 0 and x = w.
  if (x2 < 0 || x1 > w) {
    return false;
  }
  if (x1 < 0 || x2 > w) {
    // Above tests guarantee that x1 != x2 and that the line intersects x=0 or
    // x=w or both.
    double dx = x2 - x1;
    double dy = y2 - y1;
    double alpha1 = std::max(0.0,    - x1  / dx);
    double alpha2 = std::min(1.0, (w - x1) / dx);
    x2 = x1 + dx * alpha2;
    y2 = y1 + dy * alpha2;
    x1 += dx * alpha1;
    y1 += dy * alpha1;
  }

  // Recover the original vertex order.
  if (polarity) {
    SWAP(x1, x2)
    SWAP(y1, y2)
  }

  *_x1 = x1;
  *_y1 = y1;
  *_x2 = x2;
  *_y2 = y2;
  #undef SWAP
  return true;
}

//***************************************************************************
// Testing.

static double Rand() {
  return double(rand()) / double(RAND_MAX);
}

TEST_FUNCTION(Test125eN) {
  for (int i = -10; i <= 10; i++) {
    double base = pow(10, i);
    printf("Test IndexTo125eN and RoundUpTo125eN for base = %e\n", base);
    double x = base;
    CHECK(RoundUpTo125eN(x) == i * 3 + 0);
    CHECK(fabs(IndexTo125eN(RoundUpTo125eN(x)) - base     ) < 1e-15);
    x = base * 1.2;
    CHECK(RoundUpTo125eN(x) == i * 3 + 1);
    CHECK(fabs(IndexTo125eN(RoundUpTo125eN(x)) - base * 2 ) < 1e-15);
    x = base * 1.9;
    CHECK(RoundUpTo125eN(x) == i * 3 + 1);
    CHECK(fabs(IndexTo125eN(RoundUpTo125eN(x)) - base * 2 ) < 1e-15);
    x = base * 2;
    CHECK(RoundUpTo125eN(x) == i * 3 + 1);
    CHECK(fabs(IndexTo125eN(RoundUpTo125eN(x)) - base * 2 ) < 1e-15);
    x = base * 2.1;
    CHECK(RoundUpTo125eN(x) == i * 3 + 2);
    CHECK(fabs(IndexTo125eN(RoundUpTo125eN(x)) - base * 5 ) < 1e-15);
    x = base * 4.9;
    CHECK(RoundUpTo125eN(x) == i * 3 + 2);
    CHECK(fabs(IndexTo125eN(RoundUpTo125eN(x)) - base * 5 ) < 1e-15);
    x = base * 5;
    CHECK(RoundUpTo125eN(x) == i * 3 + 2);
    CHECK(fabs(IndexTo125eN(RoundUpTo125eN(x)) - base * 5 ) < 1e-15);
    x = base * 5.1;
    CHECK(RoundUpTo125eN(x) == i * 3 + 3);
    CHECK(fabs(IndexTo125eN(RoundUpTo125eN(x)) - base * 10) < 1e-15);
    x = base * 9.9;
    CHECK(RoundUpTo125eN(x) == i * 3 + 3);
    CHECK(fabs(IndexTo125eN(RoundUpTo125eN(x)) - base * 10) < 1e-15);
    x = base * 10;
    CHECK(RoundUpTo125eN(x) == i * 3 + 3);
    CHECK(fabs(IndexTo125eN(RoundUpTo125eN(x)) - base * 10) < 1e-15);
  }
}

TEST_FUNCTION(TestMajorityFloorLog10) {
  for (int iter = 0; iter < 10000; iter++) {
    double scale = pow(10, Rand() * 10 - 5);
    double v1 = scale * (Rand() * 2 - 1);
    double v2 = scale * (Rand() * 2 - 1);
    double lo = std::min(v1, v2);
    double hi = std::max(v1, v2);
    double majority = MajorityFloorLog10(lo, hi);

    // Compute the majority value by brute force.
    std::map<double, int> count;
    const int N = 5000;
    for (int i = 0; i <= N; i++) {
      double value = lo + ((hi - lo) * i) / N;
      if (value != 0) {
        count[floor(log10(fabs(value)))]++;
      }
    }
    int best_count = -1;
    double best_N = 0;
    for (std::map<double, int>::iterator it = count.begin();
         it != count.end(); ++it) {
      if (it->second > best_count) {
        best_N = it->first;
        best_count = it->second;
      }
    }

    // For debugging the test:
    //   printf("lo=%f hi=%f best_N=%.0f majority=%.0f\n",
    //          lo, hi, best_N, majority);
    CHECK(best_N == majority);
  }
}

TEST_FUNCTION(TestParseSuperSubText) {
  {
    vector<std::pair<int, string> > pieces;
    ParseSuperSubText("", &pieces);
    CHECK(pieces.size() == 0);
  }
  {
    vector<std::pair<int, string> > pieces;
    ParseSuperSubText("normal", &pieces);
    CHECK(pieces.size() == 1);
    CHECK(pieces[0] == std::make_pair( 0, string("normal")));
  }
  {
    vector<std::pair<int, string> > pieces;
    ParseSuperSubText("normal_{sub1_{subsub1}sub2}"
                      "normal2^{super1^{super2}}normal3", &pieces);
    CHECK(pieces.size() == 8);
    CHECK(pieces[0] == std::make_pair( 0, string("normal")));
    CHECK(pieces[1] == std::make_pair(-1, string("sub1")));
    CHECK(pieces[2] == std::make_pair(-2, string("subsub1")));
    CHECK(pieces[3] == std::make_pair(-1, string("sub2")));
    CHECK(pieces[4] == std::make_pair( 0, string("normal2")));
    CHECK(pieces[5] == std::make_pair( 1, string("super1")));
    CHECK(pieces[6] == std::make_pair( 2, string("super2")));
    CHECK(pieces[7] == std::make_pair( 0, string("normal3")));
  }
  {
    vector<std::pair<int, string> > pieces;
    ParseSuperSubText("normal_{_{_{sub3}}}", &pieces);
    CHECK(pieces.size() == 2);
    CHECK(pieces[0] == std::make_pair( 0, string("normal")));
    CHECK(pieces[1] == std::make_pair(-3, string("sub3")));
  }
  {
    vector<std::pair<int, string> > pieces;
    ParseSuperSubText("normal^{^{^{super3}}}normal2", &pieces);
    CHECK(pieces.size() == 3);
    CHECK(pieces[0] == std::make_pair(0, string("normal")));
    CHECK(pieces[1] == std::make_pair(3, string("super3")));
    CHECK(pieces[2] == std::make_pair(0, string("normal2")));
  }
  {
    vector<std::pair<int, string> > pieces;
    ParseSuperSubText("norm_\\{norm^\\{norm_{sub\\}sub}norm\\_{sub\\}",
                      &pieces);
    CHECK(pieces.size() == 3);
    CHECK(pieces[0] == std::make_pair( 0, string("norm_{norm^{norm")));
    CHECK(pieces[1] == std::make_pair(-1, string("sub}sub")));
    CHECK(pieces[2] == std::make_pair( 0, string("norm_{sub}")));
  }
}

//@@@ find a way to run this test and inspect the visual output
void RunAllWindowTests(Window *win, const char *block_character) {
  // Make sure that text alignment works for regular text.
  for (int orientation = 0; orientation < 2; orientation++) {
    const int ww = win->Width();
    win->UseLabelFont();
    const int w = win->TextWidth(block_character);
    const int h = win->TextHeight();
    const int xofs = w, yofs = h;
    for (int x = 0; x < 3; x++) {
      for (int y = 0; y < 3; y++) {
        TextAlignment halign = (x == 0) ? TEXT_ALIGN_LEFT :
                               (x == 1) ? TEXT_ALIGN_CENTER :
                                          TEXT_ALIGN_RIGHT;
        TextAlignment valign = (y == 0) ? TEXT_ALIGN_BOTTOM :
                               (y == 1) ? TEXT_ALIGN_CENTER :
                                          TEXT_ALIGN_TOP;
        int xpos = xofs + x*w*3, ypos = yofs + y*h*3;
        win->Color(0xff);
        if (orientation) {
          win->DrawText(block_character, ww - ypos, xpos, halign, valign, true);
        } else {
          win->DrawText(block_character, xpos, ypos, halign, valign);
        }

        // Adjust so xpos,ypos is for the bottom left.
        if (halign == TEXT_ALIGN_CENTER) {
          xpos -= w / 2;
        } else if (halign == TEXT_ALIGN_RIGHT) {
          xpos -= w - 1;
        }
        if (valign == TEXT_ALIGN_CENTER) {
          ypos -= h / 2;
        } else if (valign == TEXT_ALIGN_TOP) {
          ypos -= h - 1;
        }
        win->Color(0xff0000);
        win->StartLine(Window::GRID);
        if (orientation) {
          win->Line(ww - (ypos      ), xpos      , ww-(ypos      ), xpos + w-1);
          win->Line(ww - (ypos      ), xpos + w-1, ww-(ypos + h-1), xpos + w-1);
          win->Line(ww - (ypos + h-1), xpos + w-1, ww-(ypos + h-1), xpos      );
          win->Line(ww - (ypos + h-1), xpos      , ww-(ypos      ), xpos      );
        } else {
          win->Line(xpos, ypos, xpos + w-1, ypos);
          win->Line(xpos + w-1, ypos, xpos + w-1, ypos + h-1);
          win->Line(xpos + w-1, ypos + h-1, xpos, ypos + h-1);
          win->Line(xpos, ypos + h-1, xpos, ypos);
        }
        win->EndLine();
      }
    }
    win->Color(0xff0000);
    win->StartLine(Window::GRID);
    if (orientation) {
      win->Line(ww-h*8, 0, ww-h*8, 500);
    } else {
      win->Line(0, h*8, 500, h*8);
    }
    win->EndLine();
    win->Color(0);
    win->UseLabelSmallFont();
    win->DrawText("Blue characters in red boxes:", orientation ? ww-h*8 : w,
                  orientation ? w : h*8, TEXT_ALIGN_LEFT, TEXT_ALIGN_BASELINE,
                  orientation);
    win->UseLabelFont();
  }

  // Make sure that text alignment works for formatted text.
  {
    char s[1000];
    snprintf(s, sizeof(s), "%sNormal^{Super%s}_{Sub%s}",
             block_character, block_character, block_character);
    double w = win->FormattedTextWidth(s);
    double h = win->FormattedTextHeight(s);
    const int xofs = 200, yofs = 300;
    win->Color(0xff0000);
    win->StartLine(Window::GRID);
    win->Line(xofs-w+1, yofs, xofs+2*w, yofs);
    win->Line(xofs, yofs-h+1, xofs, yofs+h-1);
    win->Line(xofs-w+1, yofs-h+1, xofs+w-1, yofs-h+1);
    win->Line(xofs-w+1, yofs+h-1, xofs+w-1, yofs+h-1);
    win->Line(xofs+w-1, yofs-h+1, xofs+w-1, yofs+h-1);
    win->Line(xofs-w+1, yofs-h+1, xofs-w+1, yofs+h-1);
    win->EndLine();
    win->Color(0xff);
    win->DrawFormattedText(s, xofs, yofs, TEXT_ALIGN_LEFT, TEXT_ALIGN_TOP);
    win->DrawFormattedText(s, xofs, yofs, TEXT_ALIGN_LEFT, TEXT_ALIGN_BOTTOM);
    win->DrawFormattedText(s, xofs, yofs, TEXT_ALIGN_RIGHT, TEXT_ALIGN_TOP);
    win->DrawFormattedText(s, xofs, yofs, TEXT_ALIGN_RIGHT, TEXT_ALIGN_BOTTOM);
    win->DrawFormattedText(s, xofs+w-1, yofs, TEXT_ALIGN_LEFT,
                           TEXT_ALIGN_BASELINE);
  }
}

}  // namespace Plot
