package com.boc_dev.physics_library.particle_system_dynamics_verbose;

import com.boc_dev.maths.objects.vector.Vec3d;

import java.util.ArrayList;

public interface NaryForce {
	Vec3d calculateForceOnParticle(Particle targetParticle, ArrayList<Particle> particles);
}
