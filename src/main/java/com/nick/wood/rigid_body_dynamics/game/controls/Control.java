package com.nick.wood.rigid_body_dynamics.game.controls;

import com.nick.wood.rigid_body_dynamics.maths.Matrix4d;
import com.nick.wood.rigid_body_dynamics.maths.Vec3d;

public interface Control {

	void reset();
	void mouseMove(double dx, double dy, boolean shiftPressed);
	void leftLinear();
	void rightLinear();
	void forwardLinear();
	void backLinear();
	void upLinear();
	void downLinear();
	void leftRoll();
	void rightRoll();
	void upPitch();
	void downPitch();
	void leftYaw();
	void rightYaw();
	void action();

	Vec3d getLinearMomentum(Matrix4d rotation, Vec3d currentLinearMomentum);
	Vec3d getAngularMomentum(Matrix4d rotation, Vec3d currentAngularMomentum);
}
