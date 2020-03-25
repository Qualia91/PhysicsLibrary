package com.nick.wood.rigid_body_dynamics.graphics;

import com.nick.wood.rigid_body_dynamics.particle_system_dynamics_verbose.Vec3d;

public class Vertex {

	private Vec3d pos;

	public Vertex(Vec3d pos) {
		this.pos = pos;
	}

	public Vec3d getPos() {
		return pos;
	}
}
