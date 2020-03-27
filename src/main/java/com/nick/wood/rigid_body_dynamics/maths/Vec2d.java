package com.nick.wood.rigid_body_dynamics.maths;

import java.util.Objects;

public class Vec2d {

	public static final Vec2d ZERO = new Vec2d(0.0, 0.0);
	public static final Vec2d X = new Vec2d(1.0, 0.0);
	public static final Vec2d Y = new Vec2d(0.0, 1.0);

	private final double x;
	private final double y;

	public Vec2d(double x, double y) {
		this.x = x;
		this.y = y;
	}

	public double getX() {
		return x;
	}

	public double getY() {
		return y;
	}

	public Vec2d add(Vec2d vec) {
		return new Vec2d(
				this.x + vec.x,
				this.y + vec.y);
	}

	public Vec2d subtract(Vec2d vec) {
		return new Vec2d(
				this.x - vec.x,
				this.y - vec.y);
	}

	public Vec2d scale(double s) {
		return new Vec2d(
				this.x * s,
				this.y * s);
	}

	public double dot(Vec2d vec) {
		return
				this.x * vec.getX() +
				this.y * vec.getY();
	}

	public double length2() {
		return
				(this.x * this.x) +
				(this.y * this.y);
	}

	public double length() {
		return Math.sqrt(length2());
	}

	public Vec2d normalise() {
		return this.scale(this.length());
	}

	public double[] getValues() {
		return new double[] {x, y};
	}

	public Matrix4d outerProduct(Vec2d vec3d) {

		int width = 4;
		double[] elements = new double[16];

		for (int thisVecIndex = 0; thisVecIndex < this.getValues().length; thisVecIndex++) {
			for (int otherVecIndex = 0; otherVecIndex < vec3d.getValues().length; otherVecIndex++) {

				elements[thisVecIndex * 4 + otherVecIndex] = this.getValues()[thisVecIndex] * vec3d.getValues()[otherVecIndex];

			}
		}

		return new Matrix4d(elements);
	}

	public Vec2d neg() {
		return new Vec2d(
				-this.x,
				-this.y
		);
	}

	@Override
	public boolean equals(Object o) {
		if (this == o) return true;
		if (o == null || getClass() != o.getClass()) return false;
		Vec2d vec2d = (Vec2d) o;
		return Double.compare(vec2d.x, x) == 0 &&
				Double.compare(vec2d.y, y) == 0;
	}

	@Override
	public int hashCode() {
		return Objects.hash(x, y);
	}
}
