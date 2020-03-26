package com.nick.wood.rigid_body_dynamics.graphics;

import com.nick.wood.rigid_body_dynamics.graphics.utils.FileUtils;
import com.nick.wood.rigid_body_dynamics.graphics.math.Matrix4d;
import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GL20;
import org.lwjgl.system.MemoryUtil;

import java.nio.FloatBuffer;

public class Shader {

	public final String vertexFile;
	public final String fragmentFile;
	private int vertexId;
	private int fragmentId;
	private int programId;

	public Shader(String vertexFile, String fragmentFile) {
		this.vertexFile = FileUtils.loadAsString(vertexFile);
		this.fragmentFile = FileUtils.loadAsString(fragmentFile);
	}

	public void create() {

		programId = GL20.glCreateProgram();


		// create shader
		vertexId = GL20.glCreateShader(GL20.GL_VERTEX_SHADER);
		createShader(vertexId, vertexFile, "Vertex");
		fragmentId = GL20.glCreateShader(GL20.GL_FRAGMENT_SHADER);
		createShader(fragmentId, fragmentFile, "Fragment");

		GL20.glAttachShader(programId, vertexId);
		GL20.glAttachShader(programId, fragmentId);

		GL20.glLinkProgram(programId);

		if (GL20.glGetProgrami(programId, GL20.GL_LINK_STATUS) == GL11.GL_FALSE) {
			throw new RuntimeException("Program linking error: " + GL20.glGetProgramInfoLog(programId));
		}

		GL20.glValidateProgram(programId);

		if (GL20.glGetProgrami(programId, GL20.GL_VALIDATE_STATUS) == GL11.GL_FALSE) {
			throw new RuntimeException("Program validate error: " + GL20.glGetProgramInfoLog(programId));
		}

		// program now does shading so delete shaders
		GL20.glDeleteShader(vertexId);
		GL20.glDeleteShader(fragmentId);
	}

	public int getUniformLocation(String name) {
		return GL20.glGetUniformLocation(programId, name);
	}

	public void setUniform(String name, Matrix4d value) {
		FloatBuffer matrix = MemoryUtil.memAllocFloat(Matrix4d.SIZE * Matrix4d.SIZE);
		matrix.put(value.getValuesF()).flip();
		GL20.glUniformMatrix4fv(getUniformLocation(name), true, matrix);
	}

	public void bind() {
		GL20.glUseProgram(programId);
	}
	public void unbind() {
		GL20.glUseProgram(0);
	}
	public void destroy() {

		GL20.glDetachShader(programId, vertexId);
		GL20.glDetachShader(programId, fragmentId);
		GL20.glDeleteShader(vertexId);
		GL20.glDeleteShader(fragmentId);
		GL20.glDeleteProgram(programId);
	}

	private void createShader(int id, String file, String typeString) {

		// set source code of shader
		GL20.glShaderSource(id, file);

		// compile shader
		GL20.glCompileShader(id);

		// error checking
		if (GL20.glGetShaderi(id, GL20.GL_COMPILE_STATUS) == GL11.GL_FALSE) {
			throw new RuntimeException(typeString + " shader not loaded: " + GL20.glGetShaderInfoLog(id));
		}
	}

	public String getVertexFile() {
		return vertexFile;
	}

	public String getFragmentFile() {
		return fragmentFile;
	}
}
