/*! @file Interpolation.h
 *  @brief Utility functions to interpolate between two values
 *在两个值之间插值的函数
 */

#ifndef PROJECT_INTERPOLATION_H
#define PROJECT_INTERPOLATION_H

#include <math.h>
#include <assert.h>
#include <type_traits>

namespace Interpolate {

/*!
 * Linear interpolation between y0 and yf.  x is between 0 and 1
 *  y0和yf之间的线性插值。x在0和1之间
 */
template <typename y_t, typename x_t>
y_t lerp(y_t y0, y_t yf, x_t x) {
  static_assert(std::is_floating_point<x_t>::value,
                "must use floating point value");
  assert(x >= 0 && x <= 1);
  return y0 + (yf - y0) * x;
}

/*!
 * Cubic bezier interpolation between y0 and yf.  x is between 0 and 1
 */
template <typename y_t, typename x_t>
y_t cubicBezier(y_t y0, y_t yf, x_t x) {
  static_assert(std::is_floating_point<x_t>::value,
                "must use floating point value");
  assert(x >= 0 && x <= 1);

  //源码 三次贝塞尔
  y_t yDiff = yf - y0;
  x_t bezier = x * x * x + x_t(3) * (x * x * (x_t(1) - x));
  return y0 + bezier * yDiff;

  //五次多项式
  // y_t  yDiff = yf - y0;
  // x_t  a =  x_t(6) *  x * x * x *x *x ;
  // x_t  b =  -x_t(15) * x * x* x *x;
  // x_t  c =  x_t(10) * x * x *x;
  // return  a * yDiff + b * yDiff + c* yDiff + y0;

  //改进复合摆线
  // y_t  yDiff = yf - y0;
  // x_t  a = -x_t(1)/x_t(2)/M_PI*sin(x_t(2)*M_PI*x);
  //   return  a * yDiff + x * yDiff  + y0;
}

/*!
 * Cubic bezier interpolation derivative between y0 and yf.  x is between 0 and
 * 1
 */
template <typename y_t, typename x_t>
y_t cubicBezierFirstDerivative(y_t y0, y_t yf, x_t x) {
  static_assert(std::is_floating_point<x_t>::value,
                "must use floating point value");
  assert(x >= 0 && x <= 1);

  //源码 三次贝塞尔
  y_t yDiff = yf - y0;
  x_t bezier = x_t(6) * x * (x_t(1) - x);
  return bezier * yDiff;

  //五次多项式
  // y_t  yDiff = yf - y0;
  // x_t  a =  x_t(30) *  x * x * x *x  ;
  // x_t  b =  -x_t(60) * x * x* x ;
  // x_t  c =  x_t(30) * x * x;
  // return  a * yDiff + b * yDiff + c* yDiff ;

  //改进复合摆线
  // y_t  yDiff = yf - y0;
  // x_t  a =  - cos(x_t(2)*M_PI *x) ;
  // return  a * yDiff + yDiff  ;
}

/*!
 * Cubic bezier interpolation derivative between y0 and yf.  x is between 0 and
 * 1
 */
template <typename y_t, typename x_t>
y_t cubicBezierSecondDerivative(y_t y0, y_t yf, x_t x) {
  static_assert(std::is_floating_point<x_t>::value,
                "must use floating point value");
  assert(x >= 0 && x <= 1);

  //源码 三次贝塞尔
  y_t yDiff = yf - y0;
  x_t bezier = x_t(6) - x_t(12) * x;
  return bezier * yDiff;

  //五次多项式
  // y_t  yDiff = yf - y0;
  // x_t  a =  x_t(120) *  x * x * x  ;
  // x_t  b =  -x_t(180) * x * x;
  // x_t  c =  x_t(60) * x;
  // return  a * yDiff + b * yDiff + c* yDiff ;

  //改进复合摆线
  // y_t  yDiff = yf - y0;
  // x_t  a = x_t(2)* M_PI *sin(x_t(2)* M_PI *x);
  //  return  a * yDiff ;
}

}  // namespace Interpolate

#endif  // PROJECT_INTERPOLATION_H
