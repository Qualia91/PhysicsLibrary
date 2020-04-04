package com.nick.wood.rigid_body_dynamics.graphics;

import com.nick.wood.rigid_body_dynamics.maths.Vec2d;
import com.nick.wood.rigid_body_dynamics.maths.Vec2f;
import com.nick.wood.rigid_body_dynamics.maths.Vec3d;

public class Vertex {

	private Vec3d pos;
	private Vec3d col;
	private Vec2f textureCoord;

	public Vertex(Vec3d pos, Vec3d col, Vec2f textureCoord) {
		this.pos = pos;
		this.col = col;
		this.textureCoord = textureCoord;
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

	public Vec2f getTextureCoord() {
		return textureCoord;
	}

	public void setTextureCoord(Vec2f textureCoord) {
		this.textureCoord = textureCoord;
	}
}
