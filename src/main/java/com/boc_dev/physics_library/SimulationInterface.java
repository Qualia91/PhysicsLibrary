package com.boc_dev.physics_library;

import com.boc_dev.physics_library.rigid_body_dynamics_verbose.RigidBody;

import java.util.ArrayList;

public interface SimulationInterface {

	void iterate(double deltaSeconds, ArrayList<RigidBody> rigidBodies);

	ArrayList<? extends Body> getBodies();
}
