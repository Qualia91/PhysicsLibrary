package com.nick.wood.rigid_body_dynamics.graphics;

import static org.lwjgl.glfw.GLFW.glfwWindowShouldClose;

public class Game implements Runnable {

	private final Runnable update;
	private double simHerts = 1;
	private final Window window;
	private long frames = 0;

	public Game(int width,
	            int height,
	            Runnable update) {

		this.update = update;
		this.window = new Window(
				width,
				height,
				"");
	}

	@Override
	public void run() {

		window.init();

		long lastTime = System.nanoTime();

		double deltaSeconds = 0.0;

		while (!glfwWindowShouldClose(window.getWindow())) {

			frames++;

			long now = System.nanoTime();

			deltaSeconds += (now - lastTime) / 1000000000.0;

			while (deltaSeconds >= 1 / simHerts) {

				update.run();

				deltaSeconds = 0.0;

				//System.out.println(frames);

				window.setTitle("FPS: " + frames);

				frames = 0;

			}


			window.loop();
			//render(getPixelMatrix.get());

			lastTime = now;

		}

		window.destroy();

	}

}
