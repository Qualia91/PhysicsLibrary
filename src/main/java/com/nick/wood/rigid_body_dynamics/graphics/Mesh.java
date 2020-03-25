package com.nick.wood.rigid_body_dynamics.graphics;

import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GL15;
import org.lwjgl.opengl.GL20;
import org.lwjgl.opengl.GL30;
import org.lwjgl.system.MemoryUtil;

import java.nio.DoubleBuffer;
import java.nio.IntBuffer;

public class Mesh {

	private Vertex[] vertices;
	private int[] indices;
	private int vao, pbo, ibo;

	public Mesh(Vertex[] vertices, int[] indices) {
		this.vertices = vertices;
		this.indices = indices;
	}

	public void create() {
		vao = GL30.glGenVertexArrays();
		GL30.glBindVertexArray(vao);

		DoubleBuffer positionBuffer = MemoryUtil.memAllocDouble(vertices.length * 3);

		double[] posData = new double[vertices.length * 3];

		for (int i = 0; i < vertices.length; i++) {
			posData[i * 3] = vertices[i].getPos().getX();
			posData[i * 3 + 1] = vertices[i].getPos().getY();
			posData[i * 3 + 2] = vertices[i].getPos().getZ();
		}

		positionBuffer.put(posData).flip();

		pbo = GL15.glGenBuffers();
		// bind to buffer
		GL15.glBindBuffer(GL15.GL_ARRAY_BUFFER, pbo);
		// put data in
		GL15.glBufferData(GL15.GL_ARRAY_BUFFER, positionBuffer, GL15.GL_STATIC_DRAW);
		// shader stuff
		GL20.glVertexAttribPointer(0, 3, GL11.GL_DOUBLE, false, 0, 0);
		// unbind from buffer
		GL15.glBindBuffer(GL15.GL_ARRAY_BUFFER, 0);

		IntBuffer indicesBuffer = MemoryUtil.memAllocInt(indices.length);
		indicesBuffer.put(indices).flip();

		ibo = GL15.glGenBuffers();
		// bind to buffer
		GL15.glBindBuffer(GL15.GL_ELEMENT_ARRAY_BUFFER, ibo);
		// put data in
		GL15.glBufferData(GL15.GL_ELEMENT_ARRAY_BUFFER, indicesBuffer, GL15.GL_STATIC_DRAW);
		// unbind from buffer
		GL15.glBindBuffer(GL15.GL_ELEMENT_ARRAY_BUFFER, 0);


	}

	public Vertex[] getVertices() {
		return vertices;
	}

	public int[] getIndices() {
		return indices;
	}

	public int getVao() {
		return vao;
	}

	public int getPbo() {
		return pbo;
	}

	public int getIbo() {
		return ibo;
	}
}
