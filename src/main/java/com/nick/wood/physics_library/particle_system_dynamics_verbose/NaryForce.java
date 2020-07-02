package com.nick.wood.physics_library.particle_system_dynamics_verbose;

import com.nick.wood.maths.objects.vector.Vec3d;

public interface NaryForce {
	Vec3d calculateForceOnParticle(Particle ... particle);
}
