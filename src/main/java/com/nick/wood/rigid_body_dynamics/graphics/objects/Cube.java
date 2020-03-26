package com.nick.wood.rigid_body_dynamics.graphics.objects;

import com.nick.wood.rigid_body_dynamics.graphics.Mesh;
import com.nick.wood.rigid_body_dynamics.graphics.Vertex;
import com.nick.wood.rigid_body_dynamics.graphics.math.Vec3d;

public class Cube implements MeshObject {

	private final Mesh mesh = new Mesh(new Vertex[] {
			//Back face
			new Vertex(new Vec3d(-0.5,  0.5, -0.5), new Vec3d(1.0, 0.0, 0.0)),
			new Vertex(new Vec3d(-0.5, -0.5, -0.5), new Vec3d(0.0, 1.0, 0.0)),
			new Vertex(new Vec3d( 0.5, -0.5, -0.5), new Vec3d(0.0, 0.0, 1.0)),
			new Vertex(new Vec3d( 0.5,  0.5, -0.5), new Vec3d(1.0, 1.0, 0.0)),

			//Front face
			new Vertex(new Vec3d(-0.5,  0.5,  0.5), new Vec3d(1.0, 0.0, 0.0)),
			new Vertex(new Vec3d(-0.5, -0.5,  0.5), new Vec3d(0.0, 1.0, 0.0)),
			new Vertex(new Vec3d( 0.5, -0.5,  0.5), new Vec3d(0.0, 0.0, 1.0)),
			new Vertex(new Vec3d( 0.5,  0.5,  0.5), new Vec3d(1.0, 1.0, 0.0)),

			//Right face
			new Vertex(new Vec3d( 0.5,  0.5, -0.5), new Vec3d(1.0, 0.0, 0.0)),
			new Vertex(new Vec3d( 0.5, -0.5, -0.5), new Vec3d(0.0, 1.0, 0.0)),
			new Vertex(new Vec3d( 0.5, -0.5,  0.5), new Vec3d(0.0, 0.0, 1.0)),
			new Vertex(new Vec3d( 0.5,  0.5,  0.5), new Vec3d(1.0, 1.0, 0.0)),

			//Left face
			new Vertex(new Vec3d(-0.5,  0.5, -0.5), new Vec3d(1.0, 0.0, 0.0)),
			new Vertex(new Vec3d(-0.5, -0.5, -0.5), new Vec3d(0.0, 1.0, 0.0)),
			new Vertex(new Vec3d(-0.5, -0.5,  0.5), new Vec3d(0.0, 0.0, 1.0)),
			new Vertex(new Vec3d(-0.5,  0.5,  0.5), new Vec3d(1.0, 1.0, 0.0)),

			//Top face
			new Vertex(new Vec3d(-0.5f,  0.5f,  0.5f), new Vec3d(1.0, 0.0, 0.0)),
			new Vertex(new Vec3d(-0.5f,  0.5f, -0.5f), new Vec3d(0.0, 1.0, 0.0)),
			new Vertex(new Vec3d( 0.5f,  0.5f, -0.5f), new Vec3d(0.0, 0.0, 1.0)),
			new Vertex(new Vec3d( 0.5f,  0.5f,  0.5f), new Vec3d(1.0, 1.0, 0.0)),

			//Bottom face
			new Vertex(new Vec3d(-0.5f, -0.5f,  0.5f), new Vec3d(1.0, 0.0, 0.0)),
			new Vertex(new Vec3d(-0.5f, -0.5f, -0.5f), new Vec3d(0.0, 1.0, 0.0)),
			new Vertex(new Vec3d( 0.5f, -0.5f, -0.5f), new Vec3d(0.0, 0.0, 1.0)),
			new Vertex(new Vec3d( 0.5f, -0.5f,  0.5f), new Vec3d(1.0, 1.0, 0.0)),
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
			});

	public Cube() {
	}

	public Mesh getMesh() {
		return mesh;
	}
}
