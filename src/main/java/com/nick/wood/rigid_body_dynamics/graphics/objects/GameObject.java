package com.nick.wood.rigid_body_dynamics.graphics.objects;

import com.nick.wood.rigid_body_dynamics.maths.Matrix4d;
import com.nick.wood.rigid_body_dynamics.maths.Vec3d;

public interface GameObject {

	void update();

	Vec3d getPosition();
	void setPosition(Vec3d position);
	Matrix4d getRotation();
	void setRotation(Matrix4d rotation);
	Vec3d getScale();
	void setScale(Vec3d scale);
	Group getMeshGroup();
	void rotateLeft();
	void rotateRight();

}
