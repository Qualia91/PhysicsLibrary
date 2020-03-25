package com.nick.wood.rigid_body_dynamics.particle_system_dynamics_verbose;

public class Vec3d {

	public static final Vec3d ZERO = new Vec3d(0.0, 0.0, 0.0);

	private final double x;
	private final double y;
	private final double z;

	public Vec3d(double x, double y, double z) {
		this.x = x;
		this.y = y;
		this.z = z;
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

	public Vec3d add(Vec3d vec) {
		return new Vec3d(
				this.x + vec.x,
				this.y + vec.y,
				this.z + vec.z);
	}

	public Vec3d subtract(Vec3d vec) {
		return new Vec3d(
				this.x - vec.x,
				this.y - vec.y,
				this.z - vec.z);
	}

	public Vec3d add(double s) {
		return new Vec3d(
				this.x + s,
				this.y + s,
				this.z + s);
	}

	public Vec3d scale(double s) {
		return new Vec3d(
				this.x * s,
				this.y * s,
				this.z * s);
	}

	public double dot(Vec3d vec) {
		return
				this.x * vec.getX() +
				this.y * vec.getY() +
				this.z * vec.getZ();
	}

	public double length2() {
		return
				(this.x * this.x) +
				(this.y * this.y) +
				(this.z * this.z);
	}

	public double length() {
		return Math.sqrt(length2());
	}
}
