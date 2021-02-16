#include "EdgeSE3Dist.h"
#include <iostream>

namespace ORB_SLAM2 {

EdgeSE3Dist::EdgeSE3Dist()
{
}

void EdgeSE3Dist::computeError()
{
  const auto* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
  const auto* v2 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
  auto t = v1->estimate().translation()-v2->estimate().translation();

  double current_dist = t.norm();

  _error[0] = current_dist - _measurement;
}

bool EdgeSE3Dist::read(std::istream& is){
  is >> _measurement;

  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeSE3Dist::write(std::ostream& os) const {
  os << measurement() << " ";

  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}

} // end namespace

