package com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose;

import com.nick.wood.rigid_body_dynamics.maths.Matrix4d;
import com.nick.wood.rigid_body_dynamics.maths.Vec3d;
import com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.ode.RigidBodyODEReturnData;

public class RigidBodyCuboid {

	// const
	private final double mass;
	private final Matrix4d IBody;
	private final Matrix4d IBodyInv;
	private final Vec3d dimensions;
	private final double density;

	// State variables
	private Vec3d origin;
	private Matrix4d rotation;
	private Vec3d linearMomentum;
	private Vec3d angularMomentum;

	// Derived quantities
	private Matrix4d InertialTensor;
	private Matrix4d Iinv;
	private Vec3d velocity;
	private final Vec3d angularVelocity;

	// Computer quantities
	private Vec3d force;
	private Vec3d torque;

	public RigidBodyCuboid(double density, Vec3d dimensions, Vec3d origin, Matrix4d rotation, Vec3d linearMomentum, Vec3d angularMomentum) {
		this.density = density;
		this.dimensions = dimensions;
		this.mass = dimensions.length() * density;
		this.origin = origin;
		this.rotation = rotation;
		this.velocity = calcVelocity(linearMomentum, mass);
		this.angularMomentum = angularMomentum;
		double xx = dimensions.getX() * dimensions.getX();
		double yy = dimensions.getY() * dimensions.getY();
		double zz = dimensions.getZ() * dimensions.getZ();
		IBody = new Matrix4d(
				yy + zz, 0.0, 0.0, 0.0,
				0.0, xx + zz, 0.0, 0.0,
				0.0, 0.0, xx + yy, 0.0,
				0.0, 0.0, 0.0, 12/mass
		).scale(mass/12.0);
		this.IBodyInv = getIBodyInv(IBody);
		this.linearMomentum = velocity.scale(mass);
		this.InertialTensor = calcInertialTensor(rotation, IBody);
		this.Iinv = calcInertialTensor(rotation, IBodyInv);
		this.angularVelocity = calcAngularVelocity(Iinv, angularMomentum);
	}

	private Vec3d calcVelocity(Vec3d momentum, double mass) {
		return momentum.scale(1/mass);
	}

	private Vec3d calcMomentum(Vec3d velocity, double mass) {
		return velocity.scale(mass);
	}

	private Matrix4d calcInertialTensor(Matrix4d rotation, Matrix4d IBody) {
		return rotation.multiply(IBody).multiply(rotation.transpose());
	}

	private Matrix4d getIBodyInv(Matrix4d iBody) {
		return new Matrix4d(
				1.0/iBody.getValues()[0], 0.0, 0.0, 0.0,
				0.0, 1.0/iBody.getValues()[5], 0.0, 0.0,
				0.0, 0.0, 1.0/iBody.getValues()[10], 0.0,
				0.0, 0.0, 0.0, 1.0
		);
	}

	private Vec3d calcAngularVelocity(Matrix4d Iinv, Vec3d angularMomentum) {
		return Iinv.multiply(angularMomentum);
	}

	public double getMass() {
		return mass;
	}

	public Matrix4d getIBody() {
		return IBody;
	}

	public Matrix4d getIBodyInv() {
		return IBodyInv;
	}

	public Vec3d getOrigin() {
		return origin;
	}

	public Matrix4d getRotation() {
		return rotation;
	}

	public Vec3d getLinearMomentum() {
		return linearMomentum;
	}

	public Vec3d getAngularMomentum() {
		return angularMomentum;
	}

	public Matrix4d getInertialTensor() {
		return InertialTensor;
	}

	public Matrix4d getIinv() {
		return Iinv;
	}

	public Vec3d getVelocity() {
		return velocity;
	}

	public Vec3d getForce() {
		return force;
	}

	public Vec3d getTorque() {
		return torque;
	}

	public RigidBodyCuboid incrementAndCopy(RigidBodyODEReturnData increment) {
		Vec3d newX = origin.add(increment.Xdot);
		Matrix4d newRotation = rotation.add(increment.Rdot);
		// todo hacky
		newRotation.getValues()[15] = 1;
		Vec3d newMomentum = linearMomentum.add(increment.Pdot);
		Vec3d newAngularMomentum = angularMomentum.add(increment.Ldot);

		return new RigidBodyCuboid(density, dimensions, newX, newRotation, newMomentum, newAngularMomentum);
	}

	public Vec3d getAngularVelocity() {
		return angularVelocity;
	}

	public void setForce(Vec3d vec3d) {
		force = vec3d;
	}

	public void setTorque(Vec3d vec3d) {
		torque = vec3d;
	}
}
