package com.nick.wood.rigid_body_dynamics;

import com.nick.wood.graphics_library.Inputs;
import com.nick.wood.graphics_library.GameObject;
import com.nick.wood.rigid_body_dynamics.particle_system_dynamics_verbose.Plane;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.UUID;

public interface SimulationInterface {

	HashMap<UUID, GameObject> getGameObjects();

	void iterate(double deltaSeconds);

	ArrayList<Plane> getPlanes();

	Inputs getInputs();
}
