package com.nick.wood.rigid_body_dynamics.particle_system_dynamics_verbose;

public interface NaryForce {
	Vec3d calculateForceOnParticle(Particle ... particle);
}
