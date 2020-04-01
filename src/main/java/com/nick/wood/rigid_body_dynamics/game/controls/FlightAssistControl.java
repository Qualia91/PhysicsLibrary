package com.nick.wood.rigid_body_dynamics.game.controls;

import com.nick.wood.rigid_body_dynamics.maths.Matrix4d;
import com.nick.wood.rigid_body_dynamics.maths.Vec3d;

public class FlightAssistControl implements Control {

	private static final double maxSpeed = 40;
	private static final double maxRotation = 1.5;
	private double sensitivity;
	private double moveSpeed;
	private double speedLateral = 0.0;
	private double speedVertical = 0.0;
	private double speedHorizontal = 0.0;
	private double speedXRotate = 0.0;
	private double speedYRotate = 0.0;
	private double speedZRotate = 0.0;

	public FlightAssistControl(double sensitivity, double moveSpeed) {
		this.sensitivity = sensitivity;
		this.moveSpeed = moveSpeed;
	}

	public void reset() {
		speedLateral = getUpdatedSpeed(speedLateral, moveSpeed/2, moveSpeed/2);
		speedHorizontal = getUpdatedSpeed(speedHorizontal, moveSpeed/2, moveSpeed/2);
		speedVertical = getUpdatedSpeed(speedVertical, moveSpeed/2, moveSpeed/2);
		speedXRotate = getUpdatedSpeed(speedXRotate, sensitivity/2, sensitivity/2);
		speedYRotate = getUpdatedSpeed(speedYRotate, sensitivity/2, sensitivity/2);
		speedZRotate = getUpdatedSpeed(speedZRotate, sensitivity/2, sensitivity/2);
	}

	private double getUpdatedSpeed(double currentSpeed, double diffAroundZero, double reduction) {
		if (currentSpeed < -diffAroundZero) {
			currentSpeed += reduction;
		} else if (currentSpeed > diffAroundZero) {
			currentSpeed -= reduction;
		} else {
			currentSpeed = 0.0;
		}
		return currentSpeed;
	}

	public void mouseMove(double dx, double dy, boolean shiftPressed) {
		if (shiftPressed) {
			speedXRotate = addToValueWithMax(speedXRotate, -dx * sensitivity * 0.01, maxRotation);
		} else {
			speedZRotate = addToValueWithMax(speedZRotate, -dx * sensitivity * 0.01, maxRotation);
		}
		speedYRotate = addToValueWithMax(speedYRotate, dy * sensitivity * 0.01, maxRotation);
	}

	private double addToValueWithMax(double val, double add, double max) {
		if (Math.abs(val) < max) {
			return val + add;
		} else {
			return val;
		}
	}

	public void leftLinear() {
		speedLateral = addToValueWithMax(speedLateral, moveSpeed, maxSpeed);
	}
	public void rightLinear() {
		speedLateral = addToValueWithMax(speedLateral, -moveSpeed, maxSpeed);
	}
	public void forwardLinear() {
		speedHorizontal = addToValueWithMax(speedHorizontal, moveSpeed, maxSpeed);
	}
	public void backLinear() {
		speedHorizontal = addToValueWithMax(speedHorizontal, -moveSpeed, maxSpeed);
	}
	public void upLinear() {
		speedVertical = addToValueWithMax(speedVertical, moveSpeed, maxSpeed);
	}
	public void downLinear() {
		speedVertical = addToValueWithMax(speedVertical, -moveSpeed, maxSpeed);
	}

	public void leftRoll() {
		speedXRotate = addToValueWithMax(speedXRotate, -sensitivity, maxRotation);
	}
	public void rightRoll() {
		speedXRotate = addToValueWithMax(speedXRotate, sensitivity, maxRotation);
	}
	public void upPitch() {
		speedYRotate = addToValueWithMax(speedYRotate, sensitivity, maxRotation);
	}
	public void downPitch() {
		speedYRotate = addToValueWithMax(speedYRotate, -sensitivity, maxRotation);
	}
	public void leftYaw() {
		speedZRotate = addToValueWithMax(speedZRotate, sensitivity, maxRotation);
	}
	public void rightYaw() {
		speedZRotate = addToValueWithMax(speedZRotate, -sensitivity, maxRotation);
	}

	public void action() {
	}

	@Override
	public Vec3d getLinearMomentum(Matrix4d rotation, Vec3d currentLinearMomentum) {
		return rotation.multiply(Vec3d.X.scale(speedHorizontal).add(Vec3d.Y.scale(speedLateral)).add(Vec3d.Z.scale(speedVertical)));
	}

	@Override
	public Vec3d getAngularMomentum(Matrix4d rotation, Vec3d currentAngularMomentum) {
		return rotation.multiply(Vec3d.X.scale(speedXRotate).add(Vec3d.Y.scale(speedYRotate)).add(Vec3d.Z.scale(speedZRotate)));
	}
}
