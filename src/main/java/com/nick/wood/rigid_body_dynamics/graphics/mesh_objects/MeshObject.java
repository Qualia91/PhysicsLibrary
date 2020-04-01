package com.nick.wood.rigid_body_dynamics.graphics.mesh_objects;

import com.nick.wood.rigid_body_dynamics.graphics.Mesh;
import com.nick.wood.rigid_body_dynamics.maths.Matrix4d;

public interface MeshObject {
	Mesh getMesh();
	Matrix4d getTransformation();
}
