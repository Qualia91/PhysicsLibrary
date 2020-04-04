package com.nick.wood.rigid_body_dynamics.graphics.mesh_objects;

import com.nick.wood.rigid_body_dynamics.graphics.Material;
import com.nick.wood.rigid_body_dynamics.graphics.Mesh;
import com.nick.wood.rigid_body_dynamics.graphics.Vertex;
import com.nick.wood.rigid_body_dynamics.maths.Matrix4d;
import com.nick.wood.rigid_body_dynamics.maths.Vec2f;
import com.nick.wood.rigid_body_dynamics.maths.Vec3d;

public class Cube implements MeshObject {

	private Vec3d position, scale;
	private Matrix4d rotation;

	private final Mesh mesh = new Mesh(new Vertex[] {
			//Back face
			new Vertex(new Vec3d(-0.5,  0.5, -0.5), new Vec3d(1.0, 0.0, 0.0), new Vec2f(0.0f, 0.0f)),
			new Vertex(new Vec3d(-0.5, -0.5, -0.5), new Vec3d(0.0, 1.0, 0.0), new Vec2f(0.0f, 1.0f)),
			new Vertex(new Vec3d( 0.5, -0.5, -0.5), new Vec3d(0.0, 0.0, 1.0), new Vec2f(1.0f, 1.0f)),
			new Vertex(new Vec3d( 0.5,  0.5, -0.5), new Vec3d(1.0, 1.0, 0.0), new Vec2f(1.0f, 0.0f)),

			//Front face
			new Vertex(new Vec3d(-0.5,  0.5,  0.5), new Vec3d(1.0, 0.0, 0.0), new Vec2f(0.0f, 0.0f)),
			new Vertex(new Vec3d(-0.5, -0.5,  0.5), new Vec3d(0.0, 1.0, 0.0), new Vec2f(0.0f, 1.0f)),
			new Vertex(new Vec3d( 0.5, -0.5,  0.5), new Vec3d(0.0, 0.0, 1.0), new Vec2f(1.0f, 1.0f)),
			new Vertex(new Vec3d( 0.5,  0.5,  0.5), new Vec3d(1.0, 1.0, 0.0), new Vec2f(1.0f, 0.0f)),

			//Right face
			new Vertex(new Vec3d( 0.5,  0.5, -0.5), new Vec3d(1.0, 0.0, 0.0), new Vec2f(0.0f, 0.0f)),
			new Vertex(new Vec3d( 0.5, -0.5, -0.5), new Vec3d(0.0, 1.0, 0.0), new Vec2f(0.0f, 1.0f)),
			new Vertex(new Vec3d( 0.5, -0.5,  0.5), new Vec3d(0.0, 0.0, 1.0), new Vec2f(1.0f, 1.0f)),
			new Vertex(new Vec3d( 0.5,  0.5,  0.5), new Vec3d(1.0, 1.0, 0.0), new Vec2f(1.0f, 0.0f)),

			//Left face
			new Vertex(new Vec3d(-0.5,  0.5, -0.5), new Vec3d(1.0, 0.0, 0.0), new Vec2f(0.0f, 0.0f)),
			new Vertex(new Vec3d(-0.5, -0.5, -0.5), new Vec3d(0.0, 1.0, 0.0), new Vec2f(0.0f, 1.0f)),
			new Vertex(new Vec3d(-0.5, -0.5,  0.5), new Vec3d(0.0, 0.0, 1.0), new Vec2f(1.0f, 1.0f)),
			new Vertex(new Vec3d(-0.5,  0.5,  0.5), new Vec3d(1.0, 1.0, 0.0), new Vec2f(1.0f, 0.0f)),

			//Top face
			new Vertex(new Vec3d(-0.5f,  0.5f,  0.5f), new Vec3d(1.0, 0.0, 0.0), new Vec2f(0.0f, 0.0f)),
			new Vertex(new Vec3d(-0.5f,  0.5f, -0.5f), new Vec3d(0.0, 1.0, 0.0), new Vec2f(0.0f, 1.0f)),
			new Vertex(new Vec3d( 0.5f,  0.5f, -0.5f), new Vec3d(0.0, 0.0, 1.0), new Vec2f(1.0f, 1.0f)),
			new Vertex(new Vec3d( 0.5f,  0.5f,  0.5f), new Vec3d(1.0, 1.0, 0.0), new Vec2f(1.0f, 0.0f)),

			//Bottom face
			new Vertex(new Vec3d(-0.5f, -0.5f,  0.5f), new Vec3d(1.0, 0.0, 0.0), new Vec2f(0.0f, 0.0f)),
			new Vertex(new Vec3d(-0.5f, -0.5f, -0.5f), new Vec3d(0.0, 1.0, 0.0), new Vec2f(0.0f, 1.0f)),
			new Vertex(new Vec3d( 0.5f, -0.5f, -0.5f), new Vec3d(0.0, 0.0, 1.0), new Vec2f(1.0f, 1.0f)),
			new Vertex(new Vec3d( 0.5f, -0.5f,  0.5f), new Vec3d(1.0, 1.0, 0.0), new Vec2f(1.0f, 0.0f)),
	},
			new int[] {
					//Back face
					0, 1, 3,
					3, 1, 2,

					//Front face
					4, 5, 7,
					7, 5, 6,

					//Right face
					8, 9, 11,
					11, 9, 10,

					//Left face
					12, 13, 15,
					15, 13, 14,

					//Top face
					16, 17, 19,
					19, 17, 18,

					//Bottom face
					20, 21, 23,
					23, 21, 22
			},
			new Material("/textures/texture.png"));

	public Cube(Vec3d translation, Vec3d scale, Matrix4d rotation) {
		this.position = translation;
		this.scale = scale;
		this.rotation = rotation;
	}

	public Mesh getMesh() {
		return mesh;
	}

	@Override
	public Matrix4d getTransformation() {
		return Matrix4d.Transform(position, rotation, scale);
	}
}
