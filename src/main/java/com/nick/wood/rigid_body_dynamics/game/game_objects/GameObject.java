package com.nick.wood.rigid_body_dynamics.game.game_objects;

import com.nick.wood.rigid_body_dynamics.graphics.mesh_objects.MeshGroup;
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
	MeshGroup getMeshGroup();
	void rotateLeft();
	void rotateRight();

}
