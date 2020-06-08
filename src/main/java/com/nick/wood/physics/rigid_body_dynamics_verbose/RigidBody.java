package com.nick.wood.physics.rigid_body_dynamics_verbose;

import com.nick.wood.maths.objects.matrix.Matrix4d;
import com.nick.wood.maths.objects.QuaternionD;
import com.nick.wood.maths.objects.vector.Vec3d;
import com.nick.wood.physics.Body;
import com.nick.wood.physics.rigid_body_dynamics_verbose.forces.Force;
import com.nick.wood.physics.rigid_body_dynamics_verbose.ode.RigidBodyODEReturnData;

import java.util.ArrayList;
import java.util.UUID;

public class RigidBody implements Body {

	private final UUID uuid;

	private static final double LINEAR_SPEED_LIMIT = 50.0;
	private static final double ANGULAR_SPEED_LIMIT = 10.0;

	// const
	private final double mass;
	private final Matrix4d IBody;
	private final Matrix4d IBodyInv;
	private final Vec3d dimensions;
	private final RigidBodyType rigidBodyType;
	// State variables
	private Vec3d origin;
	private QuaternionD rotation;
	private Vec3d linearMomentum;
	private Vec3d angularMomentum;
	// Derived quantities
	private Matrix4d InertialTensor;
	private Matrix4d Iinv;
	private Vec3d velocity;
	private Vec3d angularVelocity;
	// Computer quantities
	private Vec3d force = Vec3d.ZERO;
	private Vec3d torque = Vec3d.ZERO;
	// Impulse
	private Vec3d pos = Vec3d.ZERO;
	private Vec3d velocityImpulse = Vec3d.ZERO;
	private Vec3d angularVelocityImpulse = Vec3d.ZERO;
	private ArrayList<Force> forces;

	public RigidBody(UUID uuid, double mass, Vec3d dimensions, Vec3d origin, QuaternionD rotation, Vec3d linearMomentum, Vec3d angularMomentum, RigidBodyType rigidBodyType, ArrayList<Force> forces) {
		this.uuid = uuid;
		this.dimensions = dimensions;
		this.forces = forces;
		this.origin = origin;
		this.rotation = rotation.normalise();
		this.linearMomentum = linearMomentum;
		this.angularMomentum = angularMomentum;
		this.rigidBodyType = rigidBodyType;
		this.mass = mass;

		switch (rigidBodyType) {
			case CUBOID:
				double xx = dimensions.getX() * dimensions.getX();
				double yy = dimensions.getY() * dimensions.getY();
				double zz = dimensions.getZ() * dimensions.getZ();
				IBody = new Matrix4d(
						yy + zz, 0.0, 0.0, 0.0,
						0.0, xx + zz, 0.0, 0.0,
						0.0, 0.0, xx + yy, 0.0,
						0.0, 0.0, 0.0, 12.0/mass
				).scale(mass/12.0);
				break;
			case SPHERE:
			case SPHERE_INNER:
				double a = dimensions.getX()/2.0;
				double b = dimensions.getY()/2.0;
				double c = dimensions.getZ()/2.0;

				double aa = a * a;
				double bb = b * b;
				double cc = c * c;

				double Ia = 0.2 * mass * (bb * cc);
				double Ib = 0.2 * mass * (aa * cc);
				double Ic = 0.2 * mass * (aa * bb);

				IBody = new Matrix4d(
						Ia, 0.0, 0.0, 0.0,
						0.0, Ib, 0.0, 0.0,
						0.0, 0.0, Ic, 0.0,
						0.0, 0.0, 0.0, 1
				);
				break;

			default:
				throw new RuntimeException("Type " + rigidBodyType + " not found.");
		}

		this.velocity = calcVelocity(linearMomentum, mass);
		this.IBodyInv = getIBodyInv(IBody);
		this.InertialTensor = calcInertialTensor(this.rotation, IBody);
		this.Iinv = calcInertialTensor(this.rotation, IBodyInv);
		this.angularVelocity = calcAngularVelocity(Iinv, angularMomentum);
	}

	private Vec3d calcVelocity(Vec3d momentum, double mass) {
		Vec3d momentumUnrestricted = momentum.scale(1 / mass);
		return new Vec3d(momentumUnrestricted, LINEAR_SPEED_LIMIT);
	}

	private Vec3d calcMomentum(Vec3d velocity, double mass) {
		return velocity.scale(mass);
	}

	private Matrix4d calcInertialTensor(QuaternionD rotationQ, Matrix4d IBody) {
		Matrix4d rot = rotationQ.toMatrix().transpose();
		return rot.multiply(IBody).multiply(rot.transpose());
	}

	private Matrix4d getIBodyInv(Matrix4d iBody) {
		return new Matrix4d(
				1.0/iBody.get(0, 0), 0.0, 0.0, 0.0,
				0.0, 1.0/iBody.get(1, 1), 0.0, 0.0,
				0.0, 0.0, 1.0/iBody.get(2, 2), 0.0,
				0.0, 0.0, 0.0, 1.0
		);
	}

	private Vec3d calcAngularVelocity(Matrix4d Iinv, Vec3d angularMomentum) {
		return new Vec3d(Iinv.multiply(angularMomentum), ANGULAR_SPEED_LIMIT);
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

	public QuaternionD getRotation() {
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

	public RigidBody incrementAndCopy(RigidBodyODEReturnData increment) {
		Vec3d newX = origin.add(increment.Xdot);
		QuaternionD newRotation = rotation.add(increment.Qdot);
		Vec3d newMomentum = linearMomentum.add(increment.Pdot);
		Vec3d newAngularMomentum = angularMomentum.add(increment.Ldot);
		return new RigidBody(this.uuid, mass, dimensions, newX, newRotation, newMomentum, newAngularMomentum, rigidBodyType, forces);
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

	public Vec3d getDimensions() {
		return dimensions;
	}

	public void setVelocity(Vec3d velocity) {
		this.velocity = velocity;
		this.linearMomentum = velocity.scale(mass);
	}

	public void setAngularVelocity(Vec3d newAngularVelocity) {
		this.angularVelocity = newAngularVelocity;
		this.angularMomentum = InertialTensor.multiply(this.angularVelocity);
	}

	public void addImpulse(Vec3d pos, Vec3d velocity, Vec3d angularVelocity) {
		this.pos = this.pos.add(pos);
		this.velocityImpulse = this.velocityImpulse.add(velocity);
		this.angularVelocityImpulse = this.angularVelocityImpulse.add(angularVelocity);
	}

	public void applyImpulse() {
		this.origin = this.origin.add(pos);
		setVelocity(this.velocity.add(velocityImpulse));
		setAngularVelocity(this.angularVelocity.add(angularVelocityImpulse));
		this.pos = Vec3d.ZERO;
		this.velocityImpulse = Vec3d.ZERO;
		this.angularVelocityImpulse = Vec3d.ZERO;
	}

	public ArrayList<Force> getForces() {
		return forces;
	}

	public UUID getUuid() {
		return uuid;
	}

	public RigidBodyType getType() {
		return rigidBodyType;
	}

	public void addForce(Vec3d additionalForce) {

		this.force = force.add(additionalForce);
	}

	public void addTorque(Vec3d additionTorque) {

		this.torque = torque.add(additionTorque);
	}

	public void slowDown() {
		Vec3d linearForce = linearMomentum.scale(mass);
		addForce(linearForce.scale(-5));

		if (angularMomentum.length2() < 5) {
			addTorque(angularMomentum.scale(-20));
		} else {
			addTorque(angularMomentum.scale(-10));
		}
	}
}
