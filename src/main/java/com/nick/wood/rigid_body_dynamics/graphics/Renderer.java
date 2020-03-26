package com.nick.wood.rigid_body_dynamics.graphics;

import com.nick.wood.rigid_body_dynamics.graphics.objects.Camera;
import com.nick.wood.rigid_body_dynamics.graphics.objects.GameObject;
import com.nick.wood.rigid_body_dynamics.graphics.math.Matrix4d;
import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GL15;
import org.lwjgl.opengl.GL30;

public class Renderer {

	private final Shader shader;
	private final Matrix4d projectionMatrix;

	public Renderer(Window window) {
		this.shader = window.getShader();
		this.projectionMatrix = window.getProjectionMatrix();
	}

	public void renderMesh(GameObject gameObject, Camera camera) {
		GL30.glBindVertexArray(gameObject.getMeshObject().getMesh().getVao());
		// enable position attribute
		GL30.glEnableVertexAttribArray(0);
		// enable colour attribute
		GL30.glEnableVertexAttribArray(1);

		GL15.glBindBuffer(GL15.GL_ELEMENT_ARRAY_BUFFER, gameObject.getMeshObject().getMesh().getIbo());
		shader.bind();

		// uniform is transformation matrix
		Matrix4d transform = Matrix4d.Transform(gameObject.getPosition(), gameObject.getRotation(), gameObject.getScale());
		Matrix4d view = Matrix4d.View(camera.getPos(), camera.getRot());
		shader.setUniform("model", transform);
		shader.setUniform("projection", projectionMatrix);
		shader.setUniform("view", view);

		GL11.glDrawElements(GL11.GL_TRIANGLES, gameObject.getMeshObject().getMesh().getIndices().length, GL11.GL_UNSIGNED_INT, 0);

		shader.unbind();
		GL15.glBindBuffer(GL15.GL_ELEMENT_ARRAY_BUFFER, 0);

		GL30.glDisableVertexAttribArray(1);
		GL30.glDisableVertexAttribArray(0);
		GL30.glBindVertexArray(0);
	}
}
