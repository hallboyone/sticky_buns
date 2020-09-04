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

#include <sticky_buns/plugins/motion/stickyBuns/stickyBuns.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>

#include <boost/algorithm/clamp.hpp>


namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::MotionModel,
                scrimmage::motion::stickyBuns,
                stickyBuns_plugin)

using boost::algorithm::clamp;

namespace scrimmage {
namespace motion {

enum ModelParams {
    X = 0,
    Y,
    O,
    Vx,
    Vy,
    Vo,
    MODEL_NUM_ITEMS
};


bool stickyBuns::init(std::map<std::string, std::string> &info,
                     std::map<std::string, std::string> &params) {
  b_lat_ = sc::get<double>("b_lat", params, b_lat_);
  b_rot_ = sc::get<double>("b_rot", params, b_rot_);
  motor_spread_ = sc::get<double>("motor_spread", params, motor_spread_);
  motor_offset_ = sc::get<double>("motor_spread", params, motor_offset_);
  motor_arm_ = sc::get<double>("motor_arm", params, motor_arm_);
  Mx_ = sc::get<double>("Mx", params, Mx_);
  My_ = sc::get<double>("My", params, My_);
  Mw_ = sc::get<double>("Mw", params, Mw_);
  
  // Declare variables for controllers
  motor_l_idx_ = vars_.declare("left_motor_power", VariableIO::Direction::In);
  motor_r_idx_ = vars_.declare("right_motor_power", VariableIO::Direction::In);
  x_.resize(MODEL_NUM_ITEMS);
  x_[X] = state_->pos()(0);
  x_[Y] = state_->pos()(1);
  x_[O] = (state_->quat()).yaw();
  x_[Vx] = state_->vel()(0);
  x_[Vy] = state_->vel()(1);
  x_[Vo] = state_->ang_vel()(2);
  return true;
}

bool stickyBuns::step(double time, double dt) {
  // Get inputs and saturate
  motor_l_ = clamp(vars_.input(motor_l_idx_), -1.0, 1.0);
  motor_r_ = clamp(vars_.input(motor_r_idx_), -1.0, 1.0);
  
  x_[X] = state_->pos()(0);
  x_[Y] = state_->pos()(1);
  x_[O] = (state_->quat()).yaw();
  x_[Vx] = state_->vel()(0);
  x_[Vy] = state_->vel()(1);
  x_[Vo] = state_->ang_vel()(2);

  //double f_x_rel = .06*(motor_r_+motor_l_)*cos(motor_offset_+motor_spread_);
  //double f_y_rel = .06*(-motor_r_+motor_l_)*sin(motor_offset_+motor_spread_);

  //double f_x = f_x_rel*cos(x_[O]) - f_y_rel*sin(x_[O]);
  //double f_y = f_x_rel*sin(x_[O]) + f_y_rel*cos(x_[O]);
  
  double f_x = motor_l_*cos(x_[O] + motor_offset_ + motor_spread_) + motor_r_*cos(x_[O] - motor_spread_ - motor_offset_);
  double f_y = motor_l_*sin(x_[O] + motor_offset_ + motor_spread_) + motor_r_*sin(x_[O] - motor_spread_ - motor_offset_);

  double a_x = Mx_*f_x - b_lat_*pow(x_[Vx],2)*((x_[Vx]>0)-(x_[Vx]<0));
  double a_y = Mx_*f_y - b_lat_*pow(x_[Vy],2)*((x_[Vy]>0)-(x_[Vy]<0));
  double a_o = Mw_*(motor_l_ - motor_r_)*sin(motor_offset_);
					
  x_[X] = x_[X] + dt*x_[Vx] + pow(dt,2)/2.*a_x;
  x_[Y] = x_[Y] + dt*x_[Vy] + pow(dt,2)/2.*a_y;
  x_[O] = fmod((x_[O] + dt*x_[Vo] + pow(dt,2)/2.*a_o), 6.28318530718);
  x_[Vx] = x_[Vx] + dt * a_x;
  x_[Vy] = x_[Vy] + dt * a_y;
  x_[Vo] = x_[Vo] + dt * a_o - b_rot_ * pow(x_[Vo], 2)*((x_[Vo]>0)-(x_[Vo]<0));

  state_->pos()(0) = x_[X];
  state_->pos()(1) = x_[Y];
  state_->vel()(0) = x_[Vx];
  state_->vel()(1) = x_[Vy];
  state_->ang_vel()(2) = x_[Vo];
 
  (state_->quat()).set((state_->quat()).roll(), (state_->quat()).pitch(), x_[O]);

  return true;
}

void stickyBuns::model(const vector_t &x , vector_t &dxdt ,
                                double t) {
  /* double xy_speed = velocity_ * cos(x[PITCH]);
    dxdt[X] = xy_speed * cos(x[YAW]);
    dxdt[Y] = xy_speed * sin(x[YAW]);
    dxdt[Z] = velocity_ * sin(x[PITCH]);
    dxdt[YAW] = turn_rate_;
    dxdt[PITCH] = pitch_rate_;*/
  
}
} // namespace motion
} // namespace scrimmage
