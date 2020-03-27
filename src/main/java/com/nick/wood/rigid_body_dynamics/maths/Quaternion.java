package com.nick.wood.rigid_body_dynamics.maths;

import java.util.Arrays;

public class Quaternion {

	private final double[] q;

	public static Quaternion Rotation(Vec3d rotationVec) {
		return new Quaternion(rotationVec.normalise());
	}

	/** In radians
	 *
	 * @param angle in radians
	 * @return
	 */
	public static Quaternion RotationX(double angle) {
		return new Quaternion(Math.cos(angle/2), Math.sin(angle/2), 0.0, 0.0);
	}
	public static Quaternion RotationY(double angle) {
		return new Quaternion(Math.cos(angle/2), 0.0, Math.sin(angle/2), 0.0);
	}
	public static Quaternion RotationZ(double angle) {
		return new Quaternion(Math.cos(angle/2), 0.0, 0.0, Math.sin(angle/2));
	}

	public Quaternion normalise() {
		return this.scale(this.len());
	}

	public Quaternion(double... q) {
		this.q = q;
	}

	public Quaternion(Vec3d vec) {
		this.q = new double[] { 0.0, vec.getX(), vec.getY(), vec.getZ() };
	}

	public Quaternion rotateVector(Quaternion vector) {
		return this.multiply(vector.multiply(this.conjugate()));
	}

	public Quaternion add(Quaternion p) {
		return new Quaternion(
				q[0] + p.getQ()[0],
				q[1] + p.getQ()[1],
				q[2] + p.getQ()[2],
				q[3] + p.getQ()[3]);
	}

	public Quaternion multiply(Quaternion p) {
		return new Quaternion(
				(q[0] * p.getQ()[0]) -  (q[1] * p.getQ()[1]) - (q[2] * p.getQ()[2]) - (q[3] * p.getQ()[3]),
				(q[0] * p.getQ()[1]) +  (q[1] * p.getQ()[0]) + (q[2] * p.getQ()[3]) - (q[3] * p.getQ()[2]),
				(q[0] * p.getQ()[2]) +  (q[2] * p.getQ()[0]) - (q[1] * p.getQ()[3]) + (q[3] * p.getQ()[1]),
				(q[0] * p.getQ()[3]) +  (q[3] * p.getQ()[0]) + (q[1] * p.getQ()[2]) - (q[2] * p.getQ()[1])
		);
	}

	public Matrix4d toMatrix() {

		this.normalise();

		double x2 = q[1] * q[1];
		double y2 = q[2] * q[2];
		double z2 = q[3] * q[3];

		double wx = q[0] * q[1];
		double wy = q[0] * q[2];
		double wz = q[0] * q[3];

		double xy = q[1] * q[2];
		double xz = q[1] * q[3];

		double yz = q[2] * q[3];

		return new Matrix4d(
				0.5 - y2 - z2, xy + xz, xz - wy, 0.0,
				xy - wz, 0.5 - x2 - z2, yz + wx, 0.0,
				xz + wy, yz + wz, 0.5 - x2 - y2, 0.0,
				0.0, 0.0, 0.0, 0.5
		).scale(2);
	}

	public Quaternion scale(double s) {
		return new Quaternion(q[0]*s, q[1]*s, q[2]*s, q[3]*s);
	}

	public Quaternion conjugate() {
		return new Quaternion(q[0], - q[1], - q[2], - q[3]);
	}

	public double len2() {
		return (q[0]*q[0]) + (q[1]*q[1]) + (q[2]*q[2]) + (q[3]*q[3]);
	}

	public double len() {
		return Math.sqrt(len2());
	}

	public Quaternion inverse() {
		return conjugate().scale(1/len());
	}

	public double[] getQ() {
		return q;
	}

	public Vec3d toVec3d() {
		return new Vec3d(q[1], q[2], q[3]);
	}

	@Override
	public String toString() {
		return Arrays.toString(q);
	}
}
