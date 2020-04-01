package com.nick.wood.rigid_body_dynamics.graphics.mesh_objects;

import com.nick.wood.rigid_body_dynamics.game.game_objects.GameObject;
import com.nick.wood.rigid_body_dynamics.maths.Matrix4d;
import com.nick.wood.rigid_body_dynamics.maths.Vec3d;

public class Camera {
	private Vec3d pos;
	private Vec3d rot;
	private double moveSpeed;
	private double sensitivity;
	private double x;
	private double z;
	private double y;

	private GameObject gameObject;

	public Camera(Vec3d pos, Vec3d rot, double moveSpeed, double sensitivity) {
		this.pos = pos;
		this.rot = rot;
		this.moveSpeed = moveSpeed;
		this.sensitivity = sensitivity;
	}

	public void attachGameObject(GameObject gameObject) {
		this.gameObject = gameObject;
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
		pos = pos.add(new Vec3d(x, y, 0.0));
	}

	public void right() {
		pos = pos.add(new Vec3d(-x, -y, 0.0));
	}

	public void forward() {
		pos = pos.add(new Vec3d(y, -x, z));
	}

	public void back() {
		pos = pos.add(new Vec3d(-y, x, -z));
	}

	public void up() {
		pos = pos.add(new Vec3d(0.0, 0.0, moveSpeed));
	}

	public void down() {
		pos = pos.add(new Vec3d(0.0, 0.0, -moveSpeed));
	}

	public void rotate(double dx, double dy) {
		rot = rot.add(new Vec3d(-dy*sensitivity, 0.0,-dx*sensitivity));

		this.x = Math.cos(Math.toRadians(rot.getZ())) * moveSpeed;
		this.y = Math.sin(Math.toRadians(rot.getZ())) * moveSpeed;
		this.z = Math.cos(Math.toRadians(rot.getX())) * moveSpeed;
	}

	public Matrix4d getView() {

		// get game object to world transformation
		if (gameObject != null) {
			Matrix4d transform = Matrix4d.InverseTransformation(gameObject.getPosition().neg(), gameObject.getRotation().transpose(), Vec3d.ONE);

			return transform.multiply(Matrix4d.View(pos, rot));
		} else {
			return Matrix4d.View(pos, rot);
		}
	}
}
