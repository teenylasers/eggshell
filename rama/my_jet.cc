
#include "../toolkit/testing.h"
#include "../toolkit/error.h"
#include "my_jet.h"

TEST_FUNCTION(SqrtDerivativeBug) {
  // We added math functions to ceres jets to be able to support
  // complex<JetNum>, however we created the bug where the number 'c' below is
  // NaN instead of zero. The problem was that the derivatives of b are NaNs
  // because of the atan2 branch cut at (complex) zero. The implementation of
  // sqrt() in *some* C++ libraries looks like polar(sqrt(abs(x)),arg(x)/2),
  // where the polar function returns NaNs if isnan(x) is true. The trouble was
  // that isnan on jets returns true if the derivative is nan. We changed the
  // behavior of the jet isnan so that it just checks the value not the
  // derivatives.

  JetNum a(0.0);               // a is 0
  JetComplex b = atan2(a, a);  // b is 0 by the definition of atan2
  JetComplex c = sqrt(b);      // c should be 0, not NaN
  CHECK(ToDouble(c.real()) == 0);
  CHECK(ToDouble(c.imag()) == 0);
}
