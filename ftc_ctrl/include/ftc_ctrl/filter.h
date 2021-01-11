#pragma once

#include <Eigen/Eigen>

namespace filter{

    class Filter {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            Filter();
            Filter(const Eigen::VectorXd num, const Eigen::VectorXd den, const Eigen::Vector3d x0);
            ~Filter();

            void update(Eigen::Vector3d& out, const Eigen::Vector3d& in);
            void init(const Eigen::VectorXd num, const Eigen::VectorXd den, const Eigen::Vector3d x0);
            bool initialized();
        private:

            Eigen::VectorXd num_;
            Eigen::VectorXd den_;
            Eigen::Vector3d state_;
            bool initialized_=false;

            Eigen::MatrixXd cache_ = Eigen::MatrixXd::Zero(3,5);
            Eigen::MatrixXd filtered_cache_ = Eigen::MatrixXd::Zero(3,5);
            Eigen::Vector3d step(Eigen::MatrixXd& u, Eigen::MatrixXd& y, const Eigen::VectorXd n, const Eigen::VectorXd d);
    };
}