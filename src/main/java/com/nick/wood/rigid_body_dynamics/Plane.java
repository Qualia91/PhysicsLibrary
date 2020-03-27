package com.nick.wood.rigid_body_dynamics;

import com.nick.wood.rigid_body_dynamics.graphics.math.Vec3d;

public class Plane {

	private final Vec3d normal;
	private final Vec3d center;

	public Plane(Vec3d center, Vec3d normal) {
		this.center = center;
		this.normal = normal.normalise();
	}

	public Vec3d getNormal() {
		return normal;
	}

	public Vec3d getCenter() {
		return center;
	}
}
