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

#include <sticky_buns/plugins/autonomy/zonedPotentials/zonedPotentials.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>

#include <iostream>
#include <limits>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy,
                scrimmage::autonomy::zonedPotentials,
                zonedPotentials_plugin)

namespace scrimmage {
namespace autonomy {

void zonedPotentials::init(std::map<std::string, std::string> &params) {
  initial_speed_ = sc::get<double>("initial_speed", params, initial_speed_);
  initial_rot_ = sc::get<double>("initial_rot", params, initial_rot_);
  repulse_max_ = sc::get<double>("repulse_max", params, repulse_max_);
  attract_max_ = sc::get<double>("attract_max", params, attract_max_);
  eq_dist_ = sc::get<double>("eq_dist", params, eq_dist_);
  eq_dist_end_ = sc::get<double>("eq_dist_end", params, eq_dist_end_);
  zone_count_ = sc::get<int>("zone_count", params, zone_count_);
  sense_buffer_ = sc::get<double>("sense_buffer", params, sense_buffer_);
  max_sense_dist_ = sc::get<double>("max_sense_dist", params, max_sense_dist_);
  attract_weight_ = sc::get<double>("attract_weight", params, attract_weight_);
  bias_x_ = sc::get<double>("bias_x", params, 0);
  bias_y_ = sc::get<double>("bias_y", params, 0);
  period_ = sc::get<double>("period", params, 0);
  growth_rate_ = sc::get<double>("growth_rate", params, 0);
  group_penalty_ = sc::get<double>("group_penalty", params, 0);
  leader_penalty_ = sc::get<double>("leader_penalty", params, 0);
  leader_damage_ = sc::get<int>("leader_damage", params, 0);
    
  force_dir_idx_ = vars_.declare("w", VariableIO::Direction::Out);
  force_mag_idx_ = vars_.declare("mag", VariableIO::Direction::Out);

  zone_spread_ = 3.14159265/(float)zone_count_;

  //Set initial values
  state_->vel()(0) = initial_speed_;
  state_->ang_vel()(2) = initial_rot_;
}

  bool zonedPotentials::step_autonomy(double t, double dt) {
    // Find nearest entity on other team. Loop through each contact, calculate
    // distance to entity, save the ID of the entity that is closest.
    double min_dist[zone_count_];
    double weight;
    int cur_zone = 0;
    double rel_angle = 0;
    
    for (uint8_t i = 0; i<zone_count_; i++){
      min_dist[i] = std::numeric_limits<double>::infinity();
    }
    
    for (auto it = contacts_->begin(); it != contacts_->end(); it++) {
      if(it->second.state()->pos()==state_->pos()){//skip ourself
	continue;
      }

      //Get the entity's zone
      rel_angle = atan2(it->second.state()->pos()(1)-state_->pos()(1),it->second.state()->pos()(0)-state_->pos()(0));
      rel_angle -= (state_->quat()).yaw();
      if(rel_angle < 0){
	rel_angle += 6.28318530718;
      }
      
      //Zone 0: 30 deg - -30 def; Corner case
      if((rel_angle > 0 && rel_angle < zone_spread_) || rel_angle > 6.28318530718 - zone_spread_){
	cur_zone = 0;
      }
      //Zone 1=>zone_count_-1
      else{
	for (int i = 1; i<zone_count_; i++){
	  if(rel_angle > zone_spread_ * (2*i-1) && rel_angle < zone_spread_ * (2*i+1)){
	    cur_zone = i;
	    i = zone_count_;
	  }
	}
      }
      
      // Calculate distance to entity
      double dist = (it->second.state()->pos() - state_->pos()).norm() - sense_buffer_;
      
      if (dist < max_sense_dist_){//If the distance is within range
	if (dist < min_dist[cur_zone]) {//and the distance is the smallest
	  min_dist[cur_zone] = dist;//Save it
	}
      }
    }

    //Relative force vectors
    force_x_ = bias_x_ * sin(period_ * pow(t/100., growth_rate_));
    force_y_ = bias_y_ * cos(period_ * pow(t/100., growth_rate_));

    double eq;
    int neighbor_count = 0;
    int neighbor_present[6] = {0,0,0,0,0,0};
    int leader = 0;
    //For each min dist, calculate the weight
    for (int i = 0; i<zone_count_; i++){
      if (i==0 || i==3){
	eq = eq_dist_end_;
      }
      else{
	eq = eq_dist_;
      }
      
      if(min_dist[i] <= max_sense_dist_){//If there was an agent in the zone
	if(min_dist[i] <= eq){//If the agent was to close or to far away
	  neighbor_count++;
	  neighbor_present[i] = 1;
	  weight = -repulse_max_/pow(eq, 2)*pow(min_dist[i]-eq, 2);
	}
	else{//If it wasn't closer than the eq point, then its further
	  neighbor_count++;
	  neighbor_present[i] = 1;
	  weight = attract_max_ * exp(-1.5*(min_dist[i]-eq))*pow(1.5*(min_dist[i]-eq), 2)/(4*exp(-2));
	}
	//Add the scalled unit vector componenets to the force vector.
	force_x_ += weight * cos(i*2*zone_spread_ + state_->quat().yaw());
	force_y_ += weight * sin(i*2*zone_spread_ + state_->quat().yaw());
      }
      else if(i==0){//leader penalty
	parent()->set_health_points(parent()->health_points() - leader_damage_);
	leader = 1;
      }
    }

    //Compute the average location of neighbors
    double s_x = 0;
    double s_y = 0;
    for(size_t i = 0; i<6; i++){
      if(neighbor_present[i]==1){
	s_x += cos(i*2*zone_spread_ + state_->quat().yaw());
	s_y += sin(i*2*zone_spread_ + state_->quat().yaw());
      }
    }
    double s_w = atan2(s_y, s_x) + 1.39626;
    force_x_ += 0.05 * cos(s_w);
    force_y_ += 0.05 * sin(s_w);
    
    double w = atan2(force_y_, force_x_) ;
    double mag = sqrt(pow(force_x_, 2)+pow(force_y_, 2)) - neighbor_count*group_penalty_;

    if(leader){
      mag -= leader_penalty_;
    }
    
    // Output relative force vectors
    vars_.output(force_dir_idx_, w);
    vars_.output(force_mag_idx_, mag);
    std::cout<<w<<", "<<mag<<"\n";
    return true;
}
} // namespace autonomy
} // namespace scrimmage
