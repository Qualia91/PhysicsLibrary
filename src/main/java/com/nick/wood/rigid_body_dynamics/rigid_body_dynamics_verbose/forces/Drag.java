package com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.forces;

import com.nick.wood.maths.objects.vector.Vec3d;
import com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.RigidBody;

public class Drag implements Force{

	private static final double DRAG_COEFF = -0.1;

	@Override
	public Vec3d actLinear(RigidBody rigidBody) {
		return rigidBody.getVelocity().scale(DRAG_COEFF);
	}

	@Override
	public Vec3d actAngular(RigidBody rigidBody) {
		return rigidBody.getAngularVelocity().scale(DRAG_COEFF);
	}
}
