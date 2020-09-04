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
#include <boost/algorithm/clamp.hpp>
#include <iostream>
#include <limits>

using std::cout;
using std::endl;
using boost::algorithm::clamp;

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
	//attract_weight_ = sc::get<double>("attract_weight", params, attract_weight_);
	bias_x_ = sc::get<double>("bias_x", params, 0);
	bias_y_ = sc::get<double>("bias_y", params, 0);
	period_ = sc::get<double>("period", params, 0);
	growth_rate_ = sc::get<double>("growth_rate", params, 0);
	group_penalty_ = sc::get<double>("group_penalty", params, 0);
	leader_penalty_ = sc::get<double>("leader_penalty", params, 0);
	leader_damage_ = sc::get<int>("leader_damage", params, 0);

	force_dir_idx_ = vars_.declare("w", VariableIO::Direction::Out);
	force_mag_idx_ = vars_.declare("mag", VariableIO::Direction::Out);

	zone_spread_ = 3.14159265 / (float)zone_count_;

	//Set initial values
	state_->vel()(0) = initial_speed_;
	state_->ang_vel()(2) = initial_rot_;
}

bool zonedPotentials::step_autonomy(double t, double dt) {
	double min_dist[zone_count_];
	int neighbor_count = distancesToNearestNeighbors(min_dist);
	//bool neighbor_present_in[6] = false;

	//Initalize force with bias
	if (t > 500) {
		force_x_ = bias_x_ * sin(period_ * pow(t / 100., growth_rate_));
		force_y_ = bias_y_ * cos(period_ * pow(t / 100., growth_rate_));
		force_x_ *= clamp((1 + neighbor_count * group_penalty_),0,2);
		force_y_ *= clamp((1 + neighbor_count * group_penalty_),0,2);
		if (min_dist[0] == 0) {
			force_x_ *= 1 + leader_penalty_;
			force_y_ *= 1 + leader_penalty_;
			parent()->set_health_points(parent()->health_points() - leader_damage_);
		}
	}
	else {
		force_x_ = 0;
		force_y_ = 0;
	}
	//For each min dist, calculate the weight
	for (int zone = 0; zone < zone_count_; zone++) {
		double weight = weightOfDist(min_dist[zone], zone);
		force_x_ += weight * cos(zone * 2 * zone_spread_ + state_->quat().yaw());
		force_y_ += weight * sin(zone * 2 * zone_spread_ + state_->quat().yaw());
	}

	double w = atan2(force_y_, force_x_) ;
	double mag = sqrt(pow(force_x_, 2) + pow(force_y_, 2));

	// Output relative force vectors
	vars_.output(force_dir_idx_, w);
	vars_.output(force_mag_idx_, mag);
	return true;
}

double zonedPotentials::weightOfDist(double dist, int zone) {
	double eq_distance = ((zone == 0 || zone == 3) ? eq_dist_end_ : eq_dist_);
	double weight = 0;

	if (dist <= max_sense_dist_) { //If there was an agent in the zone
		if (dist <= eq_distance) { //Compute repuslive force
			weight = -repulse_max_ / pow(eq_distance, 2) * pow(dist - eq_distance, 2);
		}
		else { //Compute attractive force
			weight = attract_max_ * exp(-1.5 * (dist - eq_distance)) * pow(1.5 * (dist - eq_distance), 2) / (4 * exp(-2));
		}
	}
	return weight;
}

int zonedPotentials::zoneOfNeighborAt(Eigen::Vector3d & pos) {
	double dist = (pos - state_->pos()).norm() - sense_buffer_;
	if (dist <= max_sense_dist_) {
		double rel_angle = atan2(pos(1) - state_->pos()(1), pos(0) - state_->pos()(0));
		rel_angle -= (state_->quat()).yaw();

		if (rel_angle < 0) rel_angle += 6.28318530718;

		//Zone 0: 30 deg - -30 def; Corner case
		if ((rel_angle > 0 && rel_angle < zone_spread_) || rel_angle > 6.28318530718 - zone_spread_) {
			return 0;
		}
		//Zone 1=>zone_count_-1
		else {
			for (int i = 1; i < zone_count_; i++) {
				if (rel_angle > zone_spread_ * (2 * i - 1) && rel_angle < zone_spread_ * (2 * i + 1)) {
					return i;
				}
			}
		}
	}
	return -1;
}

int zonedPotentials::distancesToNearestNeighbors(double * dists) {
	for (int i = 0; i < zone_count_; i++) {
		dists[i] = std::numeric_limits<double>::infinity();
	}

	bool active_zones[6] = {false, false, false, false, false, false};
	int cur_zone;
	for (auto it = contacts_->begin(); it != contacts_->end(); it++) {
		if (it->second.state()->pos() != state_->pos()) { //skip ourselves
			if ((cur_zone = zoneOfNeighborAt(it->second.state()->pos())) != -1) {
				double dist = (it->second.state()->pos() - state_->pos()).norm() - sense_buffer_;
				dists[cur_zone] = ((dist < dists[cur_zone]) ? dist : dists[cur_zone]);
				active_zones[cur_zone] = true;
			}
		}
	}
	int num_neighbors = 0;
	for (int i = 0; i < zone_count_; i++) {
		if (active_zones[i]) num_neighbors++;
	}
	return num_neighbors;
}

} // namespace autonomy
} // namespace scrimmage