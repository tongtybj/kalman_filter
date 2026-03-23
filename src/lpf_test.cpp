#include <kalman_filter/lpf_filter.h>
#include <Eigen/Core>
#include <iostream>

int main()
{
  {
    IirFilter iir1d(100.0, 10.0, 1);
    Eigen::VectorXd init1d(1);
    init1d << 10.0;
    iir1d.setInitValues(init1d);
    Eigen::VectorXd out1d = iir1d.filterFunction(init1d);
    std::cout << "[IIR 1D] output: " << out1d(0) << std::endl;
  }

  {
    IirFilter iir3d(100.0, 10.0, 3);
    Eigen::Vector3d init3d(1.0, 1.0, 1.0);
    iir3d.setInitValues(init3d);
    Eigen::VectorXd out3d = iir3d.filterFunction(init3d);
    std::cout << "[IIR 3D] output: "
              << out3d.transpose() << std::endl;
  }

  {
    FirFilter fir1d(0.5, 1);
    Eigen::VectorXd init1d(1);
    init1d << 10.0;
    fir1d.setInitValues(init1d);
    Eigen::VectorXd out1d = fir1d.filterFunction(init1d);
    std::cout << "[FIR 1D] output: " << out1d(0) << std::endl;
  }
  
  {
    FirFilter fir3d(0.5, 3);
    Eigen::Vector3d init3d(1.0, 1.0, 1.0);
    fir3d.setInitValues(init3d);
    Eigen::VectorXd out3d = fir3d.filterFunction(init3d);
    std::cout << "[FIR 3D] output: "
              << out3d.transpose() << std::endl;
  }

  return 0;
}
