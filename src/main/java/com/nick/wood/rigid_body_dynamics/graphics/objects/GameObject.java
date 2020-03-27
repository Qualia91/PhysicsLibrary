package com.nick.wood.rigid_body_dynamics.graphics.objects;

import com.nick.wood.rigid_body_dynamics.maths.Matrix4d;
import com.nick.wood.rigid_body_dynamics.maths.Vec3d;

public class GameObject {

	private Vec3d position, scale;
	private Matrix4d rotation;
	private final MeshObject meshObject;

	public GameObject(Vec3d position, Matrix4d rotation, Vec3d scale, MeshObject meshObject) {
		this.position = position;
		this.rotation = rotation;
		this.scale = scale;
		this.meshObject = meshObject;
	}

	public void update() {
		//position = position.add(new Vec3d(0.0, 0.0, -0.01));
		//scale = scale.add(Vec3d.Y.scale(0.0));
		//rotation = rotation.add(Vec3d.Y.scale(1));
	}

	public Vec3d getPosition() {
		return position;
	}

	public void setPosition(Vec3d position) {
		this.position = position;
	}

	public Matrix4d getRotation() {
		return rotation;
	}

	public void setRotation(Matrix4d rotation) {
		this.rotation = rotation;
	}

	public Vec3d getScale() {
		return scale;
	}

	public void setScale(Vec3d scale) {
		this.scale = scale;
	}

	public MeshObject getMeshObject() {
		return meshObject;
	}

	public void rotateLeft() {
		rotation = rotation.add(Vec3d.Y.scale(10));
		System.out.println(rotation);
	}

	public void rotateRight() {
		rotation = rotation.add(Vec3d.Y.scale(-10));
	}
}
