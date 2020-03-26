package com.nick.wood.rigid_body_dynamics.graphics.objects;

import com.nick.wood.rigid_body_dynamics.graphics.math.Matrix4d;
import com.nick.wood.rigid_body_dynamics.graphics.math.Vec3d;

public class Camera {
	private Vec3d pos;
	private Vec3d rot;
	private double moveSpeed;
	private double sensitivity;
	private double x;
	private double z;
	private double y;

	public Camera(Vec3d pos, Vec3d rot, double moveSpeed, double sensitivity) {
		this.pos = pos;
		this.rot = rot;
		this.moveSpeed = moveSpeed;
		this.sensitivity = sensitivity;
	}

	public double getMoveSpeed() {
		return moveSpeed;
	}

	public void setMoveSpeed(double moveSpeed) {
		this.moveSpeed = moveSpeed;
	}

	public Vec3d getPos() {
		return pos;
	}

	public void setPos(Vec3d pos) {
		this.pos = pos;
	}

	public Vec3d getRot() {
		return rot;
	}

	public void setRot(Vec3d rot) {
		this.rot = rot;
	}

	public void left() {
		pos = pos.add(new Vec3d(-z, 0.0, x));
	}

	public void right() {
		pos = pos.add(new Vec3d(z, 0.0, -x));
	}

	public void forward() {
		pos = pos.add(new Vec3d(-x, y, -z));
	}

	public void back() {
		pos = pos.add(new Vec3d(x, -y, z));
	}

	public void up() {
		pos = pos.add(new Vec3d(0.0, moveSpeed, 0.0));
	}

	public void down() {
		pos = pos.add(new Vec3d(0.0, -moveSpeed, 0.0));
	}

	public void rotate(double dx, double dy) {
		rot = rot.add(new Vec3d(-dy*sensitivity, -dx*sensitivity, 0.0));

		this.x = Math.sin(Math.toRadians(rot.getY())) * moveSpeed;
		this.y = Math.sin(Math.toRadians(rot.getX())) * moveSpeed;
		this.z = Math.cos(Math.toRadians(rot.getY())) * moveSpeed;
	}
}
