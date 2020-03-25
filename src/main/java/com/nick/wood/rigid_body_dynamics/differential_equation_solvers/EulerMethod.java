package com.nick.wood.rigid_body_dynamics.differential_equation_solvers;

import java.util.function.BiFunction;

public class EulerMethod implements DifferentialMethod {

	private final BiFunction<Double, Double, Double> function;

	public EulerMethod(BiFunction<Double, Double, Double> function) {
		this.function = function;
	}

	public void solve(double currentX, double stepSize, double startTime, double endTime) {

		double currentTime = startTime;

		while (currentTime < endTime) {

			System.out.println(currentTime + "," + currentX);

			currentX += takeStep(stepSize, currentX, currentTime);

			currentTime += stepSize;

		}

		System.out.println(currentTime + "," + currentX);


	}

	public double takeStep(double stepSize, double currentX, double currentTime) {
		return (stepSize * function.apply(currentX, currentTime));
	}
}
