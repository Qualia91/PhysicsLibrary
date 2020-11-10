package com.boc_dev.physics_library.rigid_body_dynamics_verbose;

import com.boc_dev.maths.objects.vector.Vec3d;

public class CollisionReturnVariable {
	private final Vec3d linearVelocity;
	private final Vec3d angularVelocity;
	private final Vec3d positionDisplacement;

	public CollisionReturnVariable(Vec3d linearVelocity, Vec3d angularVelocity, Vec3d positionDisplacement) {
		this.linearVelocity = linearVelocity;
		this.angularVelocity = angularVelocity;
		this.positionDisplacement = positionDisplacement;
	}

	public Vec3d getLinearVelocity() {
		return linearVelocity;
	}

	public Vec3d getAngularVelocity() {
		return angularVelocity;
	}

	public Vec3d getPositionDisplacement() {
		return positionDisplacement;
	}
}
