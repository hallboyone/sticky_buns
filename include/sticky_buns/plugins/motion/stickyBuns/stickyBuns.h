/*!
 * @file
 *
 * @section LICENSE
 *
 * Copyright (C) 2017 by the Georgia Tech Research Institute (GTRI)
 *
 * This file is part of SCRIMMAGE.
 *
 *   SCRIMMAGE is free software: you can redistribute it and/or modify it under
 *   the terms of the GNU Lesser General Public License as published by the
 *   Free Software Foundation, either version 3 of the License, or (at your
 *   option) any later version.
 *
 *   SCRIMMAGE is distributed in the hope that it will be useful, but WITHOUT
 *   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 *   License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
 * @author Eric Squires <eric.squires@gtri.gatech.edu>
 * @date 31 July 2017
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */
#ifndef INCLUDE_SIXPING_PLUGINS_MOTION_STICKYBUNS_STICKYBUNS_H_
#define INCLUDE_SIXPING_PLUGINS_MOTION_STICKYBUNS_STICKYBUNS_H_

#include <scrimmage/motion/MotionModel.h>

#include <map>
#include <string>

namespace scrimmage {
namespace motion {
class stickyBuns : public scrimmage::MotionModel {
 public:
    bool init(std::map<std::string, std::string> &info,
              std::map<std::string, std::string> &params) override;
    bool step(double time, double dt) override;
    void model(const vector_t &x , vector_t &dxdt , double t) override;

 protected:
    uint8_t motor_l_idx_ = 0;
    uint8_t motor_r_idx_ = 0;

    double motor_l_ = 0;
    double motor_r_ = 0;

    double b_lat_ = 0;
    double b_rot_ = 0;

    
    double motor_spread_;//One half the angle seperating the motor's
    double motor_offset_;//Angle in radians the motors are offset
    double motor_arm_;//Length in meters the motors are mounted from the CG
    double Mx_ = .25;
    double My_ = .25;
    double Mw_ = 0.5;
    
 private:
};
} // namespace motion
} // namespace scrimmage
#endif // INCLUDE_SIXPING_PLUGINS_MOTION_STICKYBUNS_STICKYBUNS_H_
