#include "simu_mas.hpp"
#include <cmath>
#include <cstdlib>

double PhysicSimulator::rand_value() {
	double val = std::rand() / RAND_MAX;
	return (val - 0.5) * 2;
}

RobotData PhysicSimulator::operator ()(const RobotData& curr, const PhysicSimuConfig& config, double dt) {
	RobotState   q  = curr.q_real;
	RobotControl u = curr.u_compute;
	
	// Add noise to the control signal
	if (config.actor_noise != 0) {
		u.v += rand_value() * config.actor_noise;
		u.w += rand_value() * config.actor_noise;
	}
	
	// Update by Physics Laws
	double cs = std::cos(q.theta);
	double ss = std::sin(q.theta);
	q.x += u.v * cs * dt;
	q.y += u.v * ss * dt;
	q.theta += u.w * dt;
	
	// Fill up next state by computed results
	RobotData next = curr;
	next.q_real = q;
	next.q_sense = q;
	next.u_apply = u;
	if (config.sensor_noise != 0) {
		next.q_sense.x += rand_value() * config.sensor_noise;
		next.q_sense.y += rand_value() * config.sensor_noise;
		next.q_sense.theta += rand_value() * config.sensor_noise;
	}
	
	return next;
}