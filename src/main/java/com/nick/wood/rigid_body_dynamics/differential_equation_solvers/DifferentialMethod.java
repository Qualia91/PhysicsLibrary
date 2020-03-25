package com.nick.wood.rigid_body_dynamics.differential_equation_solvers;

public interface DifferentialMethod {
	void solve(double currentX, double stepSize, double startTime, double endTime);
	double takeStep(double stepSize, double currentX, double currentTime);
}
