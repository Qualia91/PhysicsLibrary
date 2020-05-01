package com.nick.wood.physics.rigid_body_dynamics_verbose.forces;

import com.nick.wood.maths.objects.vector.Vec3d;
import com.nick.wood.physics.rigid_body_dynamics_verbose.RigidBody;

public class GravityBasic implements Force{

	private static final double g = 9.81;

	@Override
	public Vec3d actLinear(RigidBody rigidBody) {
		return Vec3d.Z.scale(rigidBody.getMass() * -9.81);
	}

	@Override
	public Vec3d actAngular(RigidBody rigidBody) {
		return Vec3d.ZERO;
	}
}
