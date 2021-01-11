// Copyright (C) 2021 Sihao Sun, RPG, University of Zurich, Switzerland
// Copyright (C) 2021 Davide Scaramuzza, RPG, University of Zurich, Switzerland
// 
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

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