package com.nick.wood.rigid_body_dynamics.differential_equation_solvers;

import java.util.function.BiFunction;

public class MidpointMethod implements DifferentialMethod {

	private final BiFunction<Double, Double, Double> function;

	public MidpointMethod(BiFunction<Double, Double, Double> function) {
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

		// take a euler step
		double eulerStep = stepSize * function.apply(currentX, currentTime);

		// evaluate the midpoint
		double fMid = function.apply(currentX + eulerStep, currentTime + stepSize);

		// get midStep
		return (eulerStep + fMid)/2;
	}

}
