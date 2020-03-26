package com.nick.wood.rigid_body_dynamics.graphics;

import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GL15;
import org.lwjgl.opengl.GL20;
import org.lwjgl.opengl.GL30;
import org.lwjgl.system.MemoryUtil;

import java.nio.Buffer;
import java.nio.DoubleBuffer;
import java.nio.IntBuffer;
import java.util.function.BiConsumer;
import java.util.function.BiFunction;
import java.util.function.Consumer;
import java.util.function.Function;

public class Mesh {

	private Vertex[] vertices;
	private int[] indices;
	private int vao, pbo, ibo, cbo;

	public Mesh(Vertex[] vertices, int[] indices) {
		this.vertices = vertices;
		this.indices = indices;
	}

	public void create() {
		vao = GL30.glGenVertexArrays();
		GL30.glBindVertexArray(vao);

		BiFunction<Vertex, Integer, Double> positionDataGettersBiFunc = (vertex, index) -> vertex.getPos().getValues()[index];
		double[] posData = createDataForBuffer(vertices.length * 3, positionDataGettersBiFunc);
		DoubleBuffer positionBuffer = createDoubleBufferAndPutData(vertices.length * 3, posData);
		pbo = writeDataToBuffer(GL15.GL_ARRAY_BUFFER, bufferType -> {
			GL15.glBufferData(bufferType, positionBuffer, GL15.GL_STATIC_DRAW);
			// shader stuff
			GL20.glVertexAttribPointer(0, 3, GL11.GL_DOUBLE, false, 0, 0);
		});

		BiFunction<Vertex, Integer, Double> colourDataGettersBiFunc = (vertex, index) -> vertex.getCol().getValues()[index];
		double[] colData = createDataForBuffer(vertices.length * 3, colourDataGettersBiFunc);
		DoubleBuffer colourBuffer = createDoubleBufferAndPutData(vertices.length * 3, colData);
		cbo = writeDataToBuffer(GL15.GL_ARRAY_BUFFER, bufferType -> {
			GL15.glBufferData(bufferType, colourBuffer, GL15.GL_STATIC_DRAW);
			// shader stuff
			GL20.glVertexAttribPointer(1, 3, GL11.GL_DOUBLE, false, 0, 0);
		});

		IntBuffer indicesBuffer = createIntBufferAndPutData(indices.length, indices);
		ibo = writeDataToBuffer(GL15.GL_ELEMENT_ARRAY_BUFFER, bufferType -> GL15.glBufferData(bufferType, indicesBuffer, GL15.GL_STATIC_DRAW));

	}

	public void destroy() {
		GL15.glDeleteBuffers(pbo);
		GL15.glDeleteBuffers(cbo);
		GL15.glDeleteBuffers(ibo);
		GL30.glDeleteVertexArrays(vao);
	}

	private DoubleBuffer createDoubleBufferAndPutData(int amount, double[] data) {
		DoubleBuffer buffer = MemoryUtil.memAllocDouble(amount);
		buffer.put(data).flip();
		return buffer;
	}

	private IntBuffer createIntBufferAndPutData(int amount, int[] data) {
		IntBuffer buffer = MemoryUtil.memAllocInt(amount);
		buffer.put(data).flip();
		return buffer;
	}

	private double[] createDataForBuffer(int amount, BiFunction<Vertex, Integer, Double> getDataFunctionArray) {
		double[] data = new double[amount];

		for (int i = 0; i < vertices.length; i++) {
			data[i * 3] =     getDataFunctionArray.apply(vertices[i], 0);
			data[i * 3 + 1] = getDataFunctionArray.apply(vertices[i], 1);
			data[i * 3 + 2] = getDataFunctionArray.apply(vertices[i], 2);
		}

		return data;
	}

	private int writeDataToBuffer(int bufferType, Consumer<Integer> bufferDataConsumer) {

		int bufferId = GL15.glGenBuffers();
		// bind to buffer
		GL15.glBindBuffer(bufferType, bufferId);
		// put data in
		bufferDataConsumer.accept(bufferType);
		// unbind from buffer
		GL15.glBindBuffer(bufferType, 0);

		return bufferId;
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
