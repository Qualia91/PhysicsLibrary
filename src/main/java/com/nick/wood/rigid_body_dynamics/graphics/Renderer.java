package com.nick.wood.rigid_body_dynamics.graphics;

import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GL15;
import org.lwjgl.opengl.GL30;

public class Renderer {

	private final Shader shader;

	public Renderer(Shader shader) {
		this.shader = shader;
	}

	public void renderMesh(Mesh mesh) {
		GL30.glBindVertexArray(mesh.getVao());
		GL30.glEnableVertexAttribArray(0);

		GL15.glBindBuffer(GL15.GL_ELEMENT_ARRAY_BUFFER, mesh.getIbo());
		shader.bind();

		GL11.glDrawElements(GL11.GL_TRIANGLES, mesh.getIndices().length, GL11.GL_UNSIGNED_INT, 0);

		shader.unbind();
		GL15.glBindBuffer(GL15.GL_ELEMENT_ARRAY_BUFFER, 0);

		GL30.glDisableVertexAttribArray(0);
		GL30.glBindVertexArray(0);
	}
}
