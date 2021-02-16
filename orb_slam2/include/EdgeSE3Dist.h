#ifndef G2O_GPS_EDGE_H
#define G2O_GPS_EDGE_H

#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/base_binary_edge.h"


namespace ORB_SLAM2 {

class EdgeSE3Dist : public g2o::BaseBinaryEdge<1, double, g2o::VertexSE3Expmap, g2o::VertexSE3Expmap>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSE3Dist();

    void computeError() override;

    bool read(std::istream& /*is*/) override;

    bool write(std::ostream& /*os*/) const override;
  };
} // end namespace

#endif
