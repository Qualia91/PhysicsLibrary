package com.nick.wood.rigid_body_dynamics.differential_equation_solvers;

import java.util.function.BiFunction;

public class RungeKutta implements DifferentialMethod {

	private final BiFunction<Double, Double, Double> function;

	public RungeKutta(BiFunction<Double, Double, Double> function) {
		this.function = function;
	}

	@Override
	public void solve(double currentX, double stepSize, double startTime, double endTime) {

		double currentTime = startTime;

		while (currentTime < endTime) {

			System.out.println(currentTime + "," + currentX);

			currentX += takeStep(stepSize, currentX, currentTime);

			currentTime += stepSize;

		}

		System.out.println(currentTime + "," + currentX);

	}

	@Override
	public double takeStep(double stepSize, double currentX, double currentTime) {

		// K1
		double k1 = function.apply(currentX, currentTime);

		// K2
		double k2 = function.apply(currentX + k1*stepSize/2, currentTime + stepSize/2);

		// K3
		double k3 = function.apply(currentX + k2*stepSize/2, currentTime + stepSize/2);

		// K4
		double k4 = function.apply(currentX + k3*stepSize, currentTime + stepSize);

		// step
		return ((k1 + (2*k2) + (2*k3) + k4)/6) * stepSize;
	}

}
