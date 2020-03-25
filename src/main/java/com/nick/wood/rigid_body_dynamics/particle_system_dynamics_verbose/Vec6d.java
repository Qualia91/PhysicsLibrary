package com.nick.wood.rigid_body_dynamics.particle_system_dynamics_verbose;

public class Vec6d {

	private final double x;
	private final double y;
	private final double z;
	private final double vx;
	private final double vy;
	private final double vz;

	public Vec6d(double x, double y, double z, double vx, double vy, double vz) {
		this.x = x;
		this.y = y;
		this.z = z;
		this.vx = vx;
		this.vy = vy;
		this.vz = vz;
	}

	public double getX() {
		return x;
	}

	public double getY() {
		return y;
	}

	public double getZ() {
		return z;
	}

	public double getVx() {
		return vx;
	}

	public double getVy() {
		return vy;
	}

	public double getVz() {
		return vz;
	}
}
