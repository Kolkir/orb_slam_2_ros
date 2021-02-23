#include "EdgeSE3Dist.h"

#include <iostream>
#include "isometry3d_gradients.h"
#include "isometry3d_mappings.h"

namespace ORB_SLAM2 {

EdgeSE3Dist::EdgeSE3Dist() {
  information().setIdentity();
}

void EdgeSE3Dist::setMeasurement(const g2o::Isometry3d& m) {
  _measurement = m;
  _inverseMeasurement = m.inverse();
}

void EdgeSE3Dist::computeError() {
  const auto* from = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
  const auto* to = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
  g2o::Isometry3d delta =
      _inverseMeasurement * from->estimate().inverse() * to->estimate();
  _error = g2o::internal::toVectorMQT(delta);
}

void EdgeSE3Dist::linearizeOplus() {
  const auto* from = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
  const auto* to = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
  Eigen::Isometry3d E;
  const Eigen::Isometry3d& Xi = from->estimate();
  const Eigen::Isometry3d& Xj = to->estimate();
  const Eigen::Isometry3d& Z = _measurement;
  g2o::internal::computeEdgeSE3Gradient(E, _jacobianOplusXi, _jacobianOplusXj,
                                        Z, Xi, Xj);
}

bool EdgeSE3Dist::read(std::istream& is) {
  g2o::Vector7d meas;
  for (int i = 0; i < 7; i++)
    is >> meas[i];
  setMeasurement(g2o::internal::fromVectorQT(meas));

  if (is.bad()) {
    return false;
  }
  for (int i = 0; i < information().rows() && is.good(); i++)
    for (int j = i; j < information().cols() && is.good(); j++) {
      is >> information()(i, j);
      if (i != j)
        information()(j, i) = information()(i, j);
    }
  if (is.bad()) {
    //  we overwrite the information matrix with the Identity
    information().setIdentity();
  }
  return true;
}

bool EdgeSE3Dist::write(std::ostream& os) const {
  g2o::Vector7d meas = g2o::internal::toVectorQT(_measurement);
  for (int i = 0; i < 7; i++)
    os << meas[i] << " ";
  for (int i = 0; i < information().rows(); i++)
    for (int j = i; j < information().cols(); j++) {
      os << information()(i, j) << " ";
    }
  return os.good();
}
}  // namespace ORB_SLAM2
