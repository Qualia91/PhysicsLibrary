package com.nick.wood.rigid_body_dynamics.game.controls;

import com.nick.wood.graphics_library.input.Control;
import com.nick.wood.maths.objects.matrix.Matrix4d;
import com.nick.wood.maths.objects.vector.Vec;
import com.nick.wood.maths.objects.vector.Vec3d;
import com.nick.wood.maths.objects.vector.Vecd;
import com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.RigidBody;

import java.util.UUID;

public class RigidBodyControl implements Control {

	private final double linearForce;
	private final double angularForce;
	private final UUID controlledRigidBodyUUID;
	private Vec3d force = Vec3d.ZERO;
	private Vec3d torque = Vec3d.ZERO;

	public RigidBodyControl(double linearForce, double angularForce, UUID controlledRigidBodyUUID) {
		this.linearForce = linearForce;
		this.angularForce = angularForce;
		this.controlledRigidBodyUUID = controlledRigidBodyUUID;
	}

	public void reset() {
		force = Vec3d.ZERO;
		torque = Vec3d.ZERO;
	}

	public void mouseMove(double dx, double dy, boolean shiftPressed) {
	}

	public Vec3d getForce() {
		return force;
	}

	public Vec3d getTorque() {
		return torque;
	}

	public void leftLinear() {
		force = force.add(new Vec3d(0.0, linearForce, 0.0));
	}
	public void rightLinear() {
		force = force.add(new Vec3d(0.0, -linearForce, 0.0));
	}
	public void forwardLinear() {
		force = force.add(new Vec3d(linearForce, 0.0, 0.0));
	}
	public void backLinear() {
		force = force.add(new Vec3d(-linearForce, 0.0, 0.0));
	}
	public void upLinear() {
		force = force.add(new Vec3d(0.0, 0.0, linearForce));
	}
	public void downLinear() {
		force = force.add(new Vec3d(0.0, 0.0, -linearForce));
	}

	public void leftRoll() {
		torque = torque.add(new Vec3d(-angularForce, 0.0, 0.0));
	}
	public void rightRoll() {
		torque = torque.add(new Vec3d(angularForce, 0.0, 0.0));
	}
	public void upPitch() {
		torque = torque.add(new Vec3d(0.0, angularForce, 0.0));
	}
	public void downPitch() {
		torque = torque.add(new Vec3d(0.0, -angularForce, 0.0));
	}
	public void leftYaw() {
		torque = torque.add(new Vec3d(0.0, 0.0, angularForce));
	}
	public void rightYaw() {
		torque = torque.add(new Vec3d(0.0, 0.0, -angularForce));
	}

	public void action() {
	}

	public UUID getUuid() {
		return controlledRigidBodyUUID;
	}
}
