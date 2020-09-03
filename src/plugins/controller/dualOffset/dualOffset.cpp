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

#include <sticky_buns/plugins/controller/dualOffset/dualOffset.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/parse/ParseUtils.h>

#include <iostream>
#include <limits>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Controller,
                scrimmage::controller::dualOffset,
                dualOffset_plugin)

namespace scrimmage {
namespace controller {

void dualOffset::init(std::map<std::string, std::string> &params) {
  motor_spread_ = sc::get<double>("motor_spread", params, 0.52359877559);
  motor_offset_ = sc::get<double>("motor_offset", params, 0.34906585039);
  motor_arm_ = sc::get<double>("motor_arm", params, 0.15);
			     
  force_mag_idx = vars_.declare("mag", VariableIO::Direction::In);
  force_dir_idx = vars_.declare("w", VariableIO::Direction::In);

  motor_l_idx = vars_.declare("left_motor_power", VariableIO::Direction::Out);
  motor_r_idx = vars_.declare("right_motor_power", VariableIO::Direction::Out);
}

bool dualOffset::step(double t, double dt) {
  //scale the input force vector to have a norm less than or equal to 1
  double w = vars_.input(force_dir_idx);
  double norm = vars_.input(force_mag_idx);
  double cur_o = state_->quat().yaw();
  
  //Calculate the motor inputs
  //  double motor_r = norm*sin(cur_o + motor_spread_ + motor_offset_ - w)/2.;
  //double motor_l = -norm*sin(cur_o - motor_spread_ - motor_offset_ - w)/2.;
  
  double motor_r = 1.01542*norm*sin(cur_o + motor_spread_ + motor_offset_ - w);
  double motor_l = -1.01542*norm*sin(cur_o - motor_spread_ - motor_offset_ - w);
  
  //Make sure the motor inputs are less than 1
  double max_input = fabs(fabs(motor_l) < fabs(motor_r) ? motor_r : motor_l);

  if (max_input > 1){
    motor_r /= max_input;
    motor_l /= max_input;
  }
  //Output the motor power
  vars_.output(motor_l_idx, motor_l);
  vars_.output(motor_r_idx, motor_r);
  
  return true;
}
} // namespace controller
} // namespace scrimmage
