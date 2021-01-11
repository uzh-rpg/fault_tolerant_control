#include "ftc_ctrl/filter.h"

namespace filter{

Filter::Filter() {
}

Filter::Filter(const Eigen::VectorXd num, const Eigen::VectorXd den, const Eigen::Vector3d x0)
        : num_(num),
          den_(den) {
    state_ = x0;

    initialized_ = true;
}

Filter::~Filter() {
}

void Filter::init(const Eigen::VectorXd num, const Eigen::VectorXd den, const Eigen::Vector3d x0) {
    
    num_ = num;
    den_ = den;
    state_ = x0;

    initialized_ = true;
    return;
}

bool Filter::initialized() {
  return initialized_;
}

void Filter::update(Eigen::Vector3d& out, const Eigen::Vector3d& in) {
    
    for (int i=0; i<3; i++) 
        cache_(i,0) = in(i);
    
    out = step(cache_, filtered_cache_, num_, den_);    

    return;
}

Eigen::Vector3d Filter::step(Eigen::MatrixXd& u, Eigen::MatrixXd& y, const Eigen::VectorXd n, const Eigen::VectorXd d) {   
    Eigen::Vector3d out;
    out << 0, 0, 0;

    for (int j=0; j<3; j++) {
      for (int i=0; i<5; i++) {
        out(j) += n(i) * u(j,i);
        if (i<5-1)
          out(j) -=  d(i+1) * y(j,i);
      }
      for (int i=5-1; i>0; i--) { 
        u(j,i) = u(j,i-1);
        y(j,i) = y(j,i-1);
      }
      y(j,0) = out(j);
    }
    
    return out;
  }
}