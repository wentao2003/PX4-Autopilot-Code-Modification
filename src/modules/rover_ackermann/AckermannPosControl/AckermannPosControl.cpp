/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "AckermannPosControl.hpp"

using namespace time_literals;

AckermannPosControl::AckermannPosControl(ModuleParams *parent) : ModuleParams(parent)
{
	_pure_pursuit_status_pub.advertise();
	updateParams();
}

void AckermannPosControl::updateParams()
{
	ModuleParams::updateParams();
	_max_yaw_rate = _param_ro_yaw_rate_limit.get() * M_DEG_TO_RAD_F;

	if (_param_ra_wheel_base.get() > FLT_EPSILON && _max_yaw_rate > FLT_EPSILON
	    && _param_ra_max_str_ang.get() > FLT_EPSILON) {
		_min_speed = _param_ra_wheel_base.get() * _max_yaw_rate / tanf(_param_ra_max_str_ang.get());
	}
}

void AckermannPosControl::updatePosControl()
{
	const hrt_abstime timestamp_prev = _timestamp;
	_timestamp = hrt_absolute_time();
	_dt = math::constrain(_timestamp - timestamp_prev, 1_ms, 5000_ms) * 1e-6f;

	updateSubscriptions();

	if (_vehicle_control_mode.flag_control_position_enabled && _vehicle_control_mode.flag_armed && runSanityChecks()) {
		generateVelocitySetpoint();

	}

}

void AckermannPosControl::updateSubscriptions()
{
	if (_vehicle_control_mode_sub.updated()) {
		_vehicle_control_mode_sub.copy(&_vehicle_control_mode);
	}

	if (_vehicle_attitude_sub.updated()) {
		vehicle_attitude_s vehicle_attitude{};
		_vehicle_attitude_sub.copy(&vehicle_attitude);
		_vehicle_attitude_quaternion = matrix::Quatf(vehicle_attitude.q);
		_vehicle_yaw = matrix::Eulerf(_vehicle_attitude_quaternion).psi();
	}

	if (_vehicle_local_position_sub.updated()) {
		vehicle_local_position_s vehicle_local_position{};
		_vehicle_local_position_sub.copy(&vehicle_local_position);

		if (!_global_ned_proj_ref.isInitialized()
		    || (_global_ned_proj_ref.getProjectionReferenceTimestamp() != vehicle_local_position.ref_timestamp)) {
			_global_ned_proj_ref.initReference(vehicle_local_position.ref_lat, vehicle_local_position.ref_lon,
							   vehicle_local_position.ref_timestamp);
		}

		_curr_pos_ned = Vector2f(vehicle_local_position.x, vehicle_local_position.y);
	}

	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;
		_vehicle_status_sub.copy(&vehicle_status);
		_nav_state = vehicle_status.nav_state;
	}

}

void AckermannPosControl::generateVelocitySetpoint()
{
	if (_vehicle_control_mode.flag_control_manual_enabled
	    && _vehicle_control_mode.flag_control_position_enabled) { // Position Mode
		manualPositionMode();

	} else if (_vehicle_control_mode.flag_control_offboard_enabled) { // Offboard Position Control
		if (_offboard_control_mode_sub.updated()) {
			_offboard_control_mode_sub.copy(&_offboard_control_mode);
		}

		if (_offboard_control_mode.position) {
			offboardPositionMode();

		}

	} else if (_vehicle_control_mode.flag_control_auto_enabled) { // Auto Mode
		autoPositionMode();
	}
}

void AckermannPosControl::manualPositionMode()
{
	manual_control_setpoint_s manual_control_setpoint{};

	if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
		const float speed_body_x_setpoint = math::interpolate<float>(manual_control_setpoint.throttle,
						    -1.f, 1.f, -_param_ro_speed_limit.get(), _param_ro_speed_limit.get());
		const float yaw_delta = math::interpolate<float>(math::deadzone(manual_control_setpoint.roll,
					_param_ro_yaw_stick_dz.get()), -1.f, 1.f, -_max_yaw_rate / _param_ro_yaw_p.get(),
					_max_yaw_rate / _param_ro_yaw_p.get());

		if (fabsf(yaw_delta) > FLT_EPSILON
		    || fabsf(speed_body_x_setpoint) < FLT_EPSILON) { // Closed loop yaw rate control
			_course_control = false;
			const float yaw_setpoint = matrix::wrap_pi(_vehicle_yaw + matrix::sign(speed_body_x_setpoint) * yaw_delta);
			rover_velocity_setpoint_s rover_velocity_setpoint{};
			rover_velocity_setpoint.timestamp = _timestamp;
			rover_velocity_setpoint.velocity_ned[0] = fabsf(speed_body_x_setpoint) * cosf(yaw_setpoint);
			rover_velocity_setpoint.velocity_ned[1] = fabsf(speed_body_x_setpoint) * sinf(yaw_setpoint);
			rover_velocity_setpoint.backwards = speed_body_x_setpoint < -FLT_EPSILON;
			_rover_velocity_setpoint_pub.publish(rover_velocity_setpoint);

		} else { // Course control if the steering input is zero (keep driving on a straight line)
			if (!_course_control) {
				_pos_ctl_course_direction = Vector2f(cos(_vehicle_yaw), sin(_vehicle_yaw));
				_pos_ctl_start_position_ned = _curr_pos_ned;
				_course_control = true;
			}

			// Construct a 'target waypoint' for course control s.t. it is never within the maximum lookahead of the rover
			const Vector2f start_to_curr_pos = _curr_pos_ned - _pos_ctl_start_position_ned;
			const float vector_scaling = fabsf(start_to_curr_pos * _pos_ctl_course_direction) + _param_pp_lookahd_max.get();
			const Vector2f target_waypoint_ned = _pos_ctl_start_position_ned + sign(speed_body_x_setpoint) *
							     vector_scaling * _pos_ctl_course_direction;
			pure_pursuit_status_s pure_pursuit_status{};
			pure_pursuit_status.timestamp = _timestamp;
			const float yaw_setpoint = PurePursuit::calcTargetBearing(pure_pursuit_status, _param_pp_lookahd_gain.get(),
						   _param_pp_lookahd_max.get(), _param_pp_lookahd_min.get(), target_waypoint_ned, _pos_ctl_start_position_ned,
						   _curr_pos_ned, fabsf(speed_body_x_setpoint));
			_pure_pursuit_status_pub.publish(pure_pursuit_status);
			rover_velocity_setpoint_s rover_velocity_setpoint{};
			rover_velocity_setpoint.timestamp = _timestamp;
			rover_velocity_setpoint.velocity_ned[0] = fabsf(speed_body_x_setpoint) * cosf(yaw_setpoint);
			rover_velocity_setpoint.velocity_ned[1] = fabsf(speed_body_x_setpoint) * sinf(yaw_setpoint);
			rover_velocity_setpoint.backwards = speed_body_x_setpoint < -FLT_EPSILON;
			_rover_velocity_setpoint_pub.publish(rover_velocity_setpoint);

		}
	}
}

void AckermannPosControl::offboardPositionMode()
{
	trajectory_setpoint_s trajectory_setpoint{};
	_trajectory_setpoint_sub.copy(&trajectory_setpoint);

	// Translate trajectory setpoint to rover setpoints
	const Vector2f target_waypoint_ned(trajectory_setpoint.position[0], trajectory_setpoint.position[1]);
	const float distance_to_target = (target_waypoint_ned - _curr_pos_ned).norm();

	if (target_waypoint_ned.isAllFinite() && distance_to_target > _param_nav_acc_rad.get()) {
		const float speed_setpoint = math::trajectory::computeMaxSpeedFromDistance(_param_ro_jerk_limit.get(),
					     _param_ro_decel_limit.get(), distance_to_target, 0.f);
		const float speed_body_x_setpoint = math::min(speed_setpoint, _param_ro_speed_limit.get());
		pure_pursuit_status_s pure_pursuit_status{};
		pure_pursuit_status.timestamp = _timestamp;
		const float yaw_setpoint = PurePursuit::calcTargetBearing(pure_pursuit_status, _param_pp_lookahd_gain.get(),
					   _param_pp_lookahd_max.get(), _param_pp_lookahd_min.get(), target_waypoint_ned, _curr_pos_ned,
					   _curr_pos_ned, fabsf(speed_body_x_setpoint));
		_pure_pursuit_status_pub.publish(pure_pursuit_status);
		rover_velocity_setpoint_s rover_velocity_setpoint{};
		rover_velocity_setpoint.timestamp = _timestamp;
		rover_velocity_setpoint.velocity_ned[0] = fabsf(speed_body_x_setpoint) * cosf(yaw_setpoint);
		rover_velocity_setpoint.velocity_ned[1] = fabsf(speed_body_x_setpoint) * sinf(yaw_setpoint);
		rover_velocity_setpoint.backwards = false;
		_rover_velocity_setpoint_pub.publish(rover_velocity_setpoint);

	} else {
		rover_velocity_setpoint_s rover_velocity_setpoint{};
		rover_velocity_setpoint.timestamp = _timestamp;
		rover_velocity_setpoint.velocity_ned[0] = 0.f;
		rover_velocity_setpoint.velocity_ned[1] = 0.f;
		rover_velocity_setpoint.backwards = false;
		_rover_velocity_setpoint_pub.publish(rover_velocity_setpoint);
	}
}

void AckermannPosControl::autoPositionMode()
{
	updateAutoSubscriptions();

	// Distances to waypoints
	const float distance_to_prev_wp = sqrt(powf(_curr_pos_ned(0) - _prev_wp_ned(0),
					       2) + powf(_curr_pos_ned(1) - _prev_wp_ned(1), 2));
	const float distance_to_curr_wp = sqrt(powf(_curr_pos_ned(0) - _curr_wp_ned(0),
					       2) + powf(_curr_pos_ned(1) - _curr_wp_ned(1), 2));

	if (_nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL
	    || _curr_wp_type == position_setpoint_s::SETPOINT_TYPE_LAND
	    || _curr_wp_type == position_setpoint_s::SETPOINT_TYPE_IDLE
	    || !_next_wp_ned.isAllFinite()) { // Check stopping conditions
		_mission_finished = distance_to_curr_wp < _param_nav_acc_rad.get();
	}

	if (_mission_finished) {
		rover_velocity_setpoint_s rover_velocity_setpoint{};
		rover_velocity_setpoint.timestamp = _timestamp;
		rover_velocity_setpoint.velocity_ned[0] = 0.f;
		rover_velocity_setpoint.velocity_ned[1] = 0.f;
		rover_velocity_setpoint.backwards = false;
		_rover_velocity_setpoint_pub.publish(rover_velocity_setpoint);

	} else { // Regular guidance algorithm
		const float speed_body_x_setpoint = calcSpeedSetpoint(_cruising_speed, _min_speed, distance_to_prev_wp,
						    distance_to_curr_wp,
						    _acceptance_radius, _prev_acceptance_radius, _param_ro_decel_limit.get(), _param_ro_jerk_limit.get(), _nav_state,
						    _waypoint_transition_angle, _prev_waypoint_transition_angle, _param_ro_speed_limit.get());
		pure_pursuit_status_s pure_pursuit_status{};
		pure_pursuit_status.timestamp = _timestamp;
		const float yaw_setpoint = PurePursuit::calcTargetBearing(pure_pursuit_status, _param_pp_lookahd_gain.get(),
					   _param_pp_lookahd_max.get(), _param_pp_lookahd_min.get(), _curr_wp_ned, _prev_wp_ned, _curr_pos_ned,
					   fabsf(speed_body_x_setpoint));
		_pure_pursuit_status_pub.publish(pure_pursuit_status);
		rover_velocity_setpoint_s rover_velocity_setpoint{};
		rover_velocity_setpoint.timestamp = _timestamp;
		rover_velocity_setpoint.velocity_ned[0] = fabsf(speed_body_x_setpoint) * cosf(yaw_setpoint);
		rover_velocity_setpoint.velocity_ned[1] = fabsf(speed_body_x_setpoint) * sinf(yaw_setpoint);
		rover_velocity_setpoint.backwards = false;
		_rover_velocity_setpoint_pub.publish(rover_velocity_setpoint);

	}
}

void AckermannPosControl::updateAutoSubscriptions()
{
	if (_home_position_sub.updated()) {
		home_position_s home_position{};
		_home_position_sub.copy(&home_position);
		_home_position = Vector2d(home_position.lat, home_position.lon);
	}

	if (_position_setpoint_triplet_sub.updated()) {
		updateWaypointsAndAcceptanceRadius();
	}

	if (_mission_result_sub.updated()) {
		mission_result_s mission_result{};
		_mission_result_sub.copy(&mission_result);
		_mission_finished = mission_result.finished;
	}
}

void AckermannPosControl::updateWaypointsAndAcceptanceRadius()
{
	position_setpoint_triplet_s position_setpoint_triplet{};
	_position_setpoint_triplet_sub.copy(&position_setpoint_triplet);
	_curr_wp_type = position_setpoint_triplet.current.type;

	RoverControl::globalToLocalSetpointTriplet(_curr_wp_ned, _prev_wp_ned, _next_wp_ned, position_setpoint_triplet,
			_curr_pos_ned, _home_position, _global_ned_proj_ref);

	_prev_waypoint_transition_angle = _waypoint_transition_angle;
	_waypoint_transition_angle = RoverControl::calcWaypointTransitionAngle(_prev_wp_ned, _curr_wp_ned, _next_wp_ned);

	// Update acceptance radius
	_prev_acceptance_radius = _acceptance_radius;

	if (_param_ra_acc_rad_max.get() >= _param_nav_acc_rad.get()) {
		_acceptance_radius = updateAcceptanceRadius(_waypoint_transition_angle, _param_nav_acc_rad.get(),
				     _param_ra_acc_rad_gain.get(), _param_ra_acc_rad_max.get(), _param_ra_wheel_base.get(), _param_ra_max_str_ang.get());

	} else {
		_acceptance_radius = _param_nav_acc_rad.get();
	}

	// Waypoint cruising speed
	_cruising_speed = position_setpoint_triplet.current.cruising_speed > 0.f ? math::constrain(
				  position_setpoint_triplet.current.cruising_speed, 0.f, _param_ro_speed_limit.get()) : _param_ro_speed_limit.get();
}

float AckermannPosControl::updateAcceptanceRadius(const float waypoint_transition_angle,
		const float default_acceptance_radius, const float acceptance_radius_gain,
		const float acceptance_radius_max, const float wheel_base, const float max_steer_angle)
{
	// Calculate acceptance radius s.t. the rover cuts the corner tangential to the current and next line segment
	float acceptance_radius = default_acceptance_radius;

	if (PX4_ISFINITE(_waypoint_transition_angle)) {
		const float theta = waypoint_transition_angle / 2.f;
		const float min_turning_radius = wheel_base / sinf(max_steer_angle);
		const float acceptance_radius_temp = min_turning_radius / tanf(theta);
		const float acceptance_radius_temp_scaled = acceptance_radius_gain *
				acceptance_radius_temp; // Scale geometric ideal acceptance radius to account for kinematic and dynamic effects
		acceptance_radius = math::constrain<float>(acceptance_radius_temp_scaled, default_acceptance_radius,
				    acceptance_radius_max);
	}

	// Publish updated acceptance radius
	position_controller_status_s pos_ctrl_status{};
	pos_ctrl_status.acceptance_radius = acceptance_radius;
	pos_ctrl_status.timestamp = _timestamp;
	_position_controller_status_pub.publish(pos_ctrl_status);
	return acceptance_radius;
}

float AckermannPosControl::calcSpeedSetpoint(const float cruising_speed, const float miss_speed_min,
		const float distance_to_prev_wp, const float distance_to_curr_wp, const float acc_rad,
		const float prev_acc_rad, const float max_decel, const float max_jerk, const int nav_state,
		const float waypoint_transition_angle, const float prev_waypoint_transition_angle, const float max_speed)
{
	// Catch improper values
	if (miss_speed_min < -FLT_EPSILON  || miss_speed_min > cruising_speed) {
		return cruising_speed;
	}

	// Cornering slow down effect
	if (distance_to_prev_wp <= prev_acc_rad && prev_acc_rad > FLT_EPSILON && PX4_ISFINITE(prev_waypoint_transition_angle)) {
		const float turning_circle = prev_acc_rad * tanf(prev_waypoint_transition_angle / 2.f);
		const float cornering_speed = _max_yaw_rate * turning_circle;
		return math::constrain(cornering_speed, miss_speed_min, cruising_speed);

	} else if (distance_to_curr_wp <= acc_rad && acc_rad > FLT_EPSILON && PX4_ISFINITE(waypoint_transition_angle)) {
		const float turning_circle = acc_rad * tanf(waypoint_transition_angle / 2.f);
		const float cornering_speed = _max_yaw_rate * turning_circle;
		return math::constrain(cornering_speed, miss_speed_min, cruising_speed);

	}

	// Straight line speed
	if (max_decel > FLT_EPSILON && max_jerk > FLT_EPSILON && acc_rad > FLT_EPSILON) {
		float straight_line_speed{0.f};

		if (nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL || !PX4_ISFINITE(waypoint_transition_angle)) {

			straight_line_speed = math::trajectory::computeMaxSpeedFromDistance(max_jerk,
					      max_decel, distance_to_curr_wp, 0.f);

		} else {
			const float turning_circle = acc_rad * tanf(waypoint_transition_angle / 2.f);
			float cornering_speed = _max_yaw_rate * turning_circle;
			cornering_speed = math::constrain(cornering_speed, miss_speed_min, cruising_speed);
			straight_line_speed = math::trajectory::computeMaxSpeedFromDistance(max_jerk,
					      max_decel, distance_to_curr_wp - acc_rad, cornering_speed);
		}

		return math::min(straight_line_speed, cruising_speed);

	} else {
		return cruising_speed;
	}

}

bool AckermannPosControl::runSanityChecks()
{
	bool ret = true;

	if (_param_ro_max_thr_speed.get() < FLT_EPSILON) {
		ret = false;
	}

	if (_param_ra_wheel_base.get() < FLT_EPSILON) {
		ret = false;
	}

	if (_param_ra_max_str_ang.get() < FLT_EPSILON) {
		ret = false;
	}

	if (_param_ro_speed_limit.get() < FLT_EPSILON) {
		ret = false;
	}

	if (_param_ro_yaw_p.get() < FLT_EPSILON) {
		ret = false;
	}

	_prev_param_check_passed = ret;
	return ret;
}
