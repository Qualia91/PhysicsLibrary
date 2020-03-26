package com.nick.wood.rigid_body_dynamics.graphics;

import com.nick.wood.rigid_body_dynamics.particle_system_dynamics_verbose.Simulation;

import static org.lwjgl.glfw.GLFW.glfwWindowShouldClose;

public class Game implements Runnable {

	private final Simulation simulation;
	private double simHerts = 120;
	private final Window window;
	private long frames = 0;

	public Game(int width,
	            int height,
	            Simulation simulation) {

		this.simulation = simulation;
		this.window = new Window(
				width,
				height,
				"",
				simulation.getParticles());
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

				simulation.eulerStep(deltaSeconds);

				window.updateDrawables(simulation.getParticles());

				window.setTitle("Iteration time: " + deltaSeconds);

				deltaSeconds = 0.0;

				frames = 0;

			}


			window.loop();
			//render(getPixelMatrix.get());

			lastTime = now;

		}

		window.destroy();

	}

}
