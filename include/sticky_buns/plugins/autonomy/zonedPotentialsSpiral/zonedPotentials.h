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

#ifndef INCLUDE_SIXPING_PLUGINS_AUTONOMY_ZONEDPOTENTIALS_ZONEDPOTENTIALS_H_
#define INCLUDE_SIXPING_PLUGINS_AUTONOMY_ZONEDPOTENTIALS_ZONEDPOTENTIALS_H_
#include <scrimmage/autonomy/Autonomy.h>

#include <string>
#include <map>
#include <vector>
#include <cstdint>

namespace scrimmage {
namespace autonomy {
class zonedPotentials : public scrimmage::Autonomy {
 public:
    void init(std::map<std::string, std::string> &params) override;
    bool step_autonomy(double t, double dt) override;

 protected:
    //Starting conditions of the agent
    double initial_speed_ = 0;
    double initial_rot_ = 0;

    //Max attractive and repulsive forces
    double repulse_max_ = 2;
    double attract_max_ = 1;

    //Distance between agents at which the force is 0
    double eq_dist_ = 1;
    double eq_dist_end_ = 1;

    //Number of sensing zones
    int zone_count_ = 6;

    //Half the angle of a single zone
    double zone_spread_;

    //Amount subtracted from centroid distance between agents
    double sense_buffer_ = .3;
    double max_sense_dist_ = 3.5;

    //Skew in the attraction curve. A higher number means a more aggressive response
    double attract_weight_ = 1;

    //Constant force driving the agent 
    double bias_x_;
    double bias_y_;
    double period_;
    double growth_rate_;
    
    //A slowing factor when an agent has many neighbors
    double group_penalty_ = 0;
    double leader_penalty_ = 0;
    int leader_damage_ = 0;
    
    //Force created by the potential fields. 
    double force_x_ = 0;
    double force_y_ = 0;

    uint8_t force_dir_idx_;
    uint8_t force_mag_idx_;
};
} // namespace autonomy
} // namespace scrimmage
#endif // INCLUDE_SIXPING_PLUGINS_AUTONOMY_ZONEDPOTENTIALS_ZONEDPOTENTIALS_H_
