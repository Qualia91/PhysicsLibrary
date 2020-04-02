package com.nick.wood.rigid_body_dynamics.maths;

import java.util.Objects;

public class Vec3d {

	public static final Vec3d ZERO = new Vec3d(0.0, 0.0, 0.0);
	public static final Vec3d X = new Vec3d(1.0, 0.0, 0.0);
	public static final Vec3d Y = new Vec3d(0.0, 1.0, 0.0);
	public static final Vec3d Z = new Vec3d(0.0, 0.0, 1.0);
	public static final Vec3d ONE = new Vec3d(1.0, 1.0, 1.0);

	private final double x;
	private final double y;
	private final double z;

	public Vec3d(double x, double y, double z) {
		this.x = x;
		this.y = y;
		this.z = z;
	}

	// Create a restricted vector. user for limiting momentums
	public Vec3d(Vec3d momentumUnrestricted, double linearSpeedLimit) {
		x = Math.min(momentumUnrestricted.getX(), linearSpeedLimit);
		y = Math.min(momentumUnrestricted.getY(), linearSpeedLimit);
		z = Math.min(momentumUnrestricted.getZ(), linearSpeedLimit);
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

	public Vec3d normalise() {
		if (this.length() == 0.0 ) {
			return Vec3d.ZERO;
		}
		return this.scale(1/this.length());
	}

	public double[] getValues() {
		return new double[] {x, y, z};
	}

	public Matrix4d outerProduct(Vec3d vec3d) {

		int width = 4;
		double[] elements = new double[16];

		for (int thisVecIndex = 0; thisVecIndex < this.getValues().length; thisVecIndex++) {
			for (int otherVecIndex = 0; otherVecIndex < vec3d.getValues().length; otherVecIndex++) {

				elements[thisVecIndex * 4 + otherVecIndex] = this.getValues()[thisVecIndex] * vec3d.getValues()[otherVecIndex];

			}
		}

		return new Matrix4d(elements);
	}

	public Vec3d cross(Vec3d vec3d) {
		return new Vec3d(
				this.y * vec3d.z - this.z * vec3d.y,
				this.x * vec3d.z - this.z * vec3d.x,
				this.x * vec3d.y - this.y * vec3d.x
		);
	}

	public Vec3d neg() {
		return new Vec3d(
				-this.x,
				-this.y,
				-this.z
		);
	}

	public Matrix4d star() {
		return new Matrix4d(
				0.0, -z, y, 0.0,
				z, 0.0, -x, 0.0,
				-y, x, 0.0, 0.0,
				0.0, 0.0, 0.0, 1.0
		);
	}

	@Override
	public boolean equals(Object o) {
		if (this == o) return true;
		if (o == null || getClass() != o.getClass()) return false;
		Vec3d vec3d = (Vec3d) o;
		return Double.compare(vec3d.x, x) == 0 &&
				Double.compare(vec3d.y, y) == 0 &&
				Double.compare(vec3d.z, z) == 0;
	}

	@Override
	public int hashCode() {
		return Objects.hash(x, y, z);
	}

	@Override
	public String toString() {
		return x + ", " + y + ", " + z;
	}

	public Vec3d multiply(Matrix4d m) {
		return new Vec3d(
				this.dot(m.getXVec()),
				this.dot(m.getYVec()),
				this.dot(m.getZVec())
		);
	}
}
