package com.boc_dev.physics_library.rigid_body_dynamics_verbose.forces;

import com.boc_dev.maths.objects.vector.Vec3d;
import com.boc_dev.physics_library.rigid_body_dynamics_verbose.RigidBody;

import java.util.ArrayList;

public class GravityBasic implements Force{

	private final double g;

	public GravityBasic() {
		this(9.81);
	}

	public GravityBasic(double g) {
		this.g = g;
	}

	@Override
	public Pair<Vec3d, Vec3d> act(RigidBody rigidBody, ArrayList<RigidBody> rigidBodyList) {
		return new Pair(Vec3d.Z.scale(rigidBody.getMass() * -9.81), Vec3d.ZERO);
	}
}
