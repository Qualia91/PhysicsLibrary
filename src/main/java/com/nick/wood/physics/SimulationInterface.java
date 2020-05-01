package com.nick.wood.physics;

import java.util.ArrayList;

public interface SimulationInterface {
	void iterate(double deltaSeconds);
	ArrayList<? extends Body> getBodies();
}
