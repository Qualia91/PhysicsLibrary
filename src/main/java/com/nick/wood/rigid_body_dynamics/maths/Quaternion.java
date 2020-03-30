package com.nick.wood.rigid_body_dynamics.maths;

import java.util.Arrays;

public class Quaternion {

	private final double[] q;

	public static Quaternion FromVec(double s, Vec3d vec) {
		return new Quaternion(s, vec.getX(), vec.getY(), vec.getZ());
	}

	public static Quaternion Rotation(Vec3d rotationVec) {
		return new Quaternion(rotationVec);
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
		return this.scale(1/this.len());
	}

	public Quaternion(double s, double i, double j, double k) {
		q = new double[4];
		q[0] = s;
		q[1] = i;
		q[2] = j;
		q[3] = k;
	}

	public double getS() {
		return q[0];
	}

	public double getI() {
		return q[1];
	}

	public double getJ() {
		return q[2];
	}

	public double getK() {
		return q[3];
	}

	private Quaternion(double... q) {
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
				(q[1] * p.getQ()[0]) +  (q[0] * p.getQ()[1]) + (q[2] * p.getQ()[3]) - (q[3] * p.getQ()[2]),
				(q[0] * p.getQ()[2]) -  (q[1] * p.getQ()[3]) + (q[2] * p.getQ()[0]) + (q[3] * p.getQ()[1]),
				(q[0] * p.getQ()[3]) +  (q[1] * p.getQ()[2]) - (q[2] * p.getQ()[1]) + (q[3] * p.getQ()[0])
		);
	}

	public Matrix4d toMatrix() {

		double[] qNorm = (this.normalise()).getQ();

		double q00 = qNorm[0] * qNorm[0];
		double q01 = qNorm[0] * qNorm[1];
		double q02 = qNorm[0] * qNorm[2];
		double q03 = qNorm[0] * qNorm[3];

		double q11 = qNorm[1] * qNorm[1];
		double q12 = qNorm[1] * qNorm[2];
		double q13 = qNorm[1] * qNorm[3];

		double q22 = qNorm[2] * qNorm[2];
		double q23 = qNorm[2] * qNorm[3];

		double q33 = qNorm[3] * qNorm[3];

		return new Matrix4d(
				q00 + q11 - 0.5, q12 - q03, q02 + q13, 0.0,
				q03 + q12, q00 + q22 - 0.5, q23 - q01, 0.0,
				q13 - q02, q01 + q23, q00 + q33 - 0.5, 0.0,
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

	private double[] getQ() {
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
