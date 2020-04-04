package com.nick.wood.rigid_body_dynamics.graphics;

import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GL13;
import org.newdawn.slick.opengl.Texture;
import org.newdawn.slick.opengl.TextureLoader;

import java.io.File;
import java.io.IOException;

public class Material {

	private Texture texture;
	public float width, height;
	private int textureId;
	private String path;

	public Material(String path) {
		this.path = path;
	}

	public void create() {
		try {
			texture = TextureLoader.getTexture(path.split("[.]")[1], Material.class.getResourceAsStream(path), GL11.GL_NEAREST);
		} catch (IOException e) {
			System.err.println("Cant find texture at " + path);
		}
		width = texture.getWidth();
		height = texture.getHeight();
		textureId = texture.getTextureID();

	}

	public void destroy() {
		GL13.glDeleteTextures(textureId);
	}

	public Texture getTexture() {
		return texture;
	}

	public float getWidth() {
		return width;
	}

	public float getHeight() {
		return height;
	}

	public int getTextureId() {
		return textureId;
	}
}
