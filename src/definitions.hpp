#ifndef DEFINITIONS_HPP
#define DEFINITIONS_HPP

#include <Eigen/Core>

using namespace Eigen;

//#define DOUBLE_PRECISION

#ifdef DOUBLE_PRECISION

#define FLOAT double
#define VEC2 Vector2d
#define VEC3 Vector3d
#define VEC4 Vector4d
#define VECX VectorXd
#define MAT2 Matrix2d
#define MAT3 Matrix3d
#define MAT4 Matrix4d
#define MATX MatrixXd
#define ANGLE_AXIS AngleAxisd
#define QUATERNION Quaterniond
#define COMPLEX std::complex<double>
#define VEC2C Vector2cd
#define VECXC VectorXcd
#define MATXC MatrixXcd

#else

#define FLOAT float
#define VEC2 Vector2f
#define VEC3 Vector3f
#define VEC4 Vector4f
#define VECX VectorXf
#define MAT2 Matrix2f
#define MAT3 Matrix3f
#define MAT4 Matrix4f
#define MATX MatrixXf
#define ANGLE_AXIS AngleAxisf
#define QUATERNION Quaternionf
#define COMPLEX std::complex<float>
#define VEC2C Vector2cf
#define VECXC VectorXcf
#define MATXC MatrixXcf

#endif

inline FLOAT damping(FLOAT d_coef, FLOAT x, FLOAT k) {
  return exp(-d_coef*k*k*x); 
}
  


inline COMPLEX fund_solution(FLOAT x) {
  return COMPLEX(0, -1)/(FLOAT)4.0*sqrtf(2.0f/(M_PI*x))*exp(COMPLEX(0, 1)*(x - (FLOAT)M_PI/4.0f));
}

inline FLOAT omega(FLOAT k) {
  return sqrtf(9.81*k + 0.074/1000*pow(k, 3));
}

inline FLOAT velocity(FLOAT k) {
  return 0.5*omega(k)/k;
}
inline FLOAT velocity(FLOAT k, FLOAT omega) {
  return 0.5*omega/k;
}

// linear interpolation 1 if x = p1, 0 if x = p2
inline FLOAT interpolation(float x, float p1, float p2) {
  float d = p2 - p1;
  float dx = x - p1;
  return 1-dx/d;
}

#endif
