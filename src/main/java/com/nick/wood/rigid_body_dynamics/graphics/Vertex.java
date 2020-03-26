package com.nick.wood.rigid_body_dynamics.graphics;

import com.nick.wood.rigid_body_dynamics.graphics.math.Vec3d;

public class Vertex {

	private Vec3d pos;
	private Vec3d col;

	public Vertex(Vec3d pos, Vec3d col) {
		this.pos = pos;
		this.col = col;
	}

	public Vec3d getPos() {
		return pos;
	}

	public void setPos(Vec3d pos) {
		this.pos = pos;
	}

	public Vec3d getCol() {
		return col;
	}

	public void setCol(Vec3d col) {
		this.col = col;
	}
}
