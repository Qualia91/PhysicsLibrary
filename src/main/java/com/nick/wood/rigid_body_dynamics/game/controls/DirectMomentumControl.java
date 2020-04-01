package com.nick.wood.rigid_body_dynamics.game.controls;

import com.nick.wood.rigid_body_dynamics.maths.Vec3d;

public class DirectMomentumControl {

	private double sensitivity;
	private double moveSpeed;
	private Vec3d linearMomentumImpulse = Vec3d.ZERO;
	private Vec3d angularMomentumImpulse = Vec3d.ZERO;

	public DirectMomentumControl(double sensitivity, double moveSpeed) {
		this.sensitivity = sensitivity;
		this.moveSpeed = moveSpeed;
	}

	public void resetImpulses() {
		linearMomentumImpulse = Vec3d.ZERO;
		angularMomentumImpulse = Vec3d.ZERO;
	}

	public Vec3d getLinearMomentumImpulse() {
		return linearMomentumImpulse;
	}

	public Vec3d getAngularMomentumImpulse() {
		return angularMomentumImpulse;
	}

	public void mouseMove(double dx, double dy) {
		angularMomentumImpulse = new Vec3d(0.0, -dy*sensitivity*0.001,-dx*sensitivity*0.001);
	}

	public void leftLinear() {
		linearMomentumImpulse = Vec3d.Y.scale(moveSpeed);
	}
	public void rightLinear() {
		linearMomentumImpulse = Vec3d.Y.scale(-moveSpeed);
	}
	public void forwardLinear() {
		linearMomentumImpulse = Vec3d.X.scale(moveSpeed);
	}
	public void backLinear() {
		linearMomentumImpulse = Vec3d.X.scale(-moveSpeed);
	}
	public void upLinear() {
		//linearMomentumImpulse = Vec3d.X.scale(moveSpeed);
	}
	public void downLinear() {
		//linearMomentumImpulse = Vec3d.X.scale(-moveSpeed);
	}

	public void leftAngular() {
		angularMomentumImpulse = Vec3d.Z.scale(sensitivity);
	}
	public void rightAngular() {
		angularMomentumImpulse = Vec3d.Z.scale(-sensitivity);
	}
	public void upAngular() {
		angularMomentumImpulse = Vec3d.Y.scale(sensitivity);
	}
	public void downAngular() {
		angularMomentumImpulse = Vec3d.Y.scale(-sensitivity);
	}

	public void action() {
		//uuidRigidBodyHashMap.get(playerRigidBodyUUID).resetLinearMomentum();
		//uuidRigidBodyHashMap.get(playerRigidBodyUUID).resetAngularMomentum();
	}
}
