package com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose;

import com.nick.wood.maths.objects.matrix.Matrix4d;
import com.nick.wood.maths.objects.vector.Vec3d;
import com.nick.wood.maths.objects.vector.Vecd;

import java.util.ArrayList;

public class CollisionDetection {

	private static final double Cr = 0.5;
	private static final double FRICTION = 0.5;

	public void collisionDetection(ArrayList<RigidBody> rigidBodyList) {

		for (int i = 0; i < rigidBodyList.size() - 1; i++) {
			for (int j = i+1; j < rigidBodyList.size(); j++) {
				if (i != j) {
					generalCollision(rigidBodyList.get(i), rigidBodyList.get(j));
				}
			}
		}
	}

	private void generalCollision(RigidBody rigidBody, RigidBody otherBody) {

		// if both spheres, use sphere collision
		if (rigidBody.getType().equals(RigidBodyType.SPHERE) && otherBody.getType().equals(RigidBodyType.SPHERE)) {
			sphereCollision(rigidBody, otherBody);
		}

		else if (rigidBody.getType().equals(RigidBodyType.SPHERE_INNER) && otherBody.getType().equals(RigidBodyType.SPHERE)) {
			sphereInnerCollision(rigidBody, otherBody);
		}
		else if (rigidBody.getType().equals(RigidBodyType.SPHERE) && otherBody.getType().equals(RigidBodyType.SPHERE_INNER)) {
			sphereInnerCollision(otherBody, rigidBody);
		}

		else if (rigidBody.getType().equals(RigidBodyType.SPHERE_INNER) && otherBody.getType().equals(RigidBodyType.CUBOID)) {
			innerSphereCuboidCollision(otherBody, rigidBody);
		}
		else if (rigidBody.getType().equals(RigidBodyType.CUBOID) && otherBody.getType().equals(RigidBodyType.SPHERE_INNER)) {
			innerSphereCuboidCollision(rigidBody, otherBody);
		}

		// if one sphere and one cuboid
		else if (rigidBody.getType().equals(RigidBodyType.CUBOID) && otherBody.getType().equals(RigidBodyType.SPHERE)) {
			sphereCuboidCollision(rigidBody, otherBody);
		}
		else if (otherBody.getType().equals(RigidBodyType.CUBOID) && rigidBody.getType().equals(RigidBodyType.SPHERE)) {
			sphereCuboidCollision(otherBody, rigidBody);
		}

		// if both cuboids
		else if (rigidBody.getType().equals(RigidBodyType.CUBOID) && otherBody.getType().equals(RigidBodyType.CUBOID)) {
			cuboidCollision(rigidBody, otherBody);
		}
	}

	private void cuboidCollision(RigidBody rigidBody, RigidBody otherBody) {

	}

	private void sphereCuboidCollision(RigidBody cuboid, RigidBody sphere) {

		// get world space to cuboid transformation
		Matrix4d inverseTransformation = Matrix4d.Translation(cuboid.getOrigin().neg()).multiply(cuboid.getRotation().inverse().toMatrix());

		// transform sphere origin from world to cuboid space
		Vec3d sphereOriginInCuboidSpace = inverseTransformation.multiply(sphere.getOrigin());

		double sphereRadius = sphere.getDimensions().getX()/2.0;

		// find the distance from center of sphere away from box
		Vec3d closestPointInAabb = Vec3d.Min(Vec3d.Max(sphereOriginInCuboidSpace, cuboid.getDimensions().scale(-0.5)), cuboid.getDimensions().scale(0.5));
		double distance = closestPointInAabb.subtract(sphereOriginInCuboidSpace).length();

		double collisionDistance = distance - sphereRadius;

		if (collisionDistance <= 0.0) {

			double ma = cuboid.getMass();
			double mb = sphere.getMass();

			// for a cube, the difference in velocity, rotated by cube rotation is normal the face of impact, ie n

			// initial velocities
			Vec3d va1 = cuboid.getVelocity();
			Vec3d vb1 = sphere.getVelocity();

			// distance vec from center of mass of A (Cube) to point of impact
			Vec3d rap = closestPointInAabb;

			// normal of cuboid face
			Vec3d n = cuboid.getRotation().toMatrix().multiply(sphereOriginInCuboidSpace.subtract(closestPointInAabb)).normalise();

			// distance vec from center of mass of B (Sphere) to point of impact
			Vec3d rbp = n.neg();


			// initial angular velocities
			Vec3d wa1 = cuboid.getAngularVelocity();
			Vec3d wb1 = sphere.getAngularVelocity();

			// initial velocity of impact point on A
			Vec3d vap1 = va1.add(wa1.cross(rap));
			// initial velocity of impact point on B
			Vec3d vbp1 = vb1.add(wb1.cross(rbp));

			// initial relative velocity of point on A
			Vec3d vabDiff = va1.subtract(vb1);
			// initial relative velocity of point on B
			Vec3d vba1 = vbp1.subtract(vap1);

			// intertial tensor inverse
			Matrix4d IaInv = cuboid.getIinv();
			Matrix4d IbInv = sphere.getIinv();

			// impulse param j
			double jal = -(1.0+Cr) *  (cuboid.getVelocity().subtract(sphere.getVelocity()).dot(n)) / (
					(1.0/ma) +
							(1.0/mb)// +
							//((IaInv.multiply(rap.cross(n)).cross(rap)).add((IbInv.multiply(rbp.cross(n)).cross(rbp)))).dot(n)
			);
			double jaa = -(1.0+Cr) *  (cuboid.getVelocity().subtract(sphere.getVelocity()).dot(n)) / (
					(1.0/ma) +
							(1.0/mb) +
							((IaInv.multiply(rap.cross(n)).cross(rap)).add((IbInv.multiply(rbp.cross(n)).cross(rbp)))).dot(n)
			);
			double jba = -(1.0+Cr) * (sphere.getVelocity().subtract(cuboid.getVelocity()).dot(n)) / (
					(1.0/ma) +
							(1.0/mb) +
							((IaInv.multiply(rap.cross(n)).cross(rap)).add((IbInv.multiply(rbp.cross(n)).cross(rbp)))).dot(n)
			);

			// now work out friction parts
			// direction of friction linear
			Vec3d tal = (n.cross(vabDiff)).cross(n).normalise();

			Vec3d taa = (n.cross(vba1)).cross(n).normalise();

			// angular velocity impulse
			Vec3d la2f = IaInv.multiply(rap.cross((n.scale(jaa).add(taa.scale(FRICTION)))));
			Vec3d lb2f = IbInv.multiply(rbp.cross((n.scale(jba).add(taa.scale(FRICTION)))));

			// velocity impulse
			Vec3d va2 = (n.scale(jal/ma).add(tal.scale(FRICTION/ma)));
			Vec3d vb2 = (n.scale(jal/mb).subtract(tal.scale(FRICTION/mb)));

			// add impulse
			cuboid.addImpulse(rbp.normalise().scale(-collisionDistance/2.0), va2, la2f);
			sphere.addImpulse(rbp.normalise().scale(collisionDistance/2.0), vb2.neg(), lb2f.neg());

		}

	}

	private void innerSphereCuboidCollision(RigidBody cuboid, RigidBody sphere) {

		// get world space to cuboid transformation
		Matrix4d inverseTransformation = Matrix4d.Translation(cuboid.getOrigin().neg()).multiply(cuboid.getRotation().inverse().toMatrix());

		// transform sphere origin from world to cuboid space
		Vec3d sphereOriginInCuboidSpace = inverseTransformation.multiply(sphere.getOrigin());

		double sphereRadius = sphere.getDimensions().getX()/2.0;

		// find the distance from center of sphere away from box
		Vec3d closestPointInAabb = Vec3d.Min(Vec3d.Max(sphereOriginInCuboidSpace, cuboid.getDimensions().scale(-0.5)), cuboid.getDimensions().scale(0.5));
		double distance = closestPointInAabb.subtract(sphereOriginInCuboidSpace).length();

		double collisionDistance = distance - sphereRadius;

		if (collisionDistance > 0.0) {

			double ma = cuboid.getMass();
			double mb = sphere.getMass();

			// for a cube, the difference in velocity, rotated by cube rotation is normal the face of impact, ie n

			// initial velocities
			Vec3d va1 = cuboid.getVelocity();
			Vec3d vb1 = sphere.getVelocity();

			// distance vec from center of mass of A (Cube) to point of impact
			Vec3d rap = closestPointInAabb;

			// normal of cuboid face
			Vec3d n = cuboid.getRotation().toMatrix().multiply(sphereOriginInCuboidSpace.subtract(closestPointInAabb)).normalise();

			// distance vec from center of mass of B (Sphere) to point of impact
			Vec3d rbp = n.neg();


			// initial angular velocities
			Vec3d wa1 = cuboid.getAngularVelocity();
			Vec3d wb1 = sphere.getAngularVelocity();

			// initial velocity of impact point on A
			Vec3d vap1 = va1.add(wa1.cross(rap));
			// initial velocity of impact point on B
			Vec3d vbp1 = vb1.add(wb1.cross(rbp));

			// initial relative velocity of point on A
			Vec3d vabDiff = va1.subtract(vb1);
			// initial relative velocity of point on B
			Vec3d vba1 = vbp1.subtract(vap1);

			// intertial tensor inverse
			Matrix4d IaInv = cuboid.getIinv();
			Matrix4d IbInv = sphere.getIinv();

			// impulse param j
			double jal = -(1.0+Cr) *  (cuboid.getVelocity().subtract(sphere.getVelocity()).dot(n)) / (
					(1.0/ma) +
							(1.0/mb)// +
					//((IaInv.multiply(rap.cross(n)).cross(rap)).add((IbInv.multiply(rbp.cross(n)).cross(rbp)))).dot(n)
			);
			double jaa = -(1.0+Cr) *  (cuboid.getVelocity().subtract(sphere.getVelocity()).dot(n)) / (
					(1.0/ma) +
							(1.0/mb) +
							((IaInv.multiply(rap.cross(n)).cross(rap)).add((IbInv.multiply(rbp.cross(n)).cross(rbp)))).dot(n)
			);
			double jba = -(1.0+Cr) * (sphere.getVelocity().subtract(cuboid.getVelocity()).dot(n)) / (
					(1.0/ma) +
							(1.0/mb) +
							((IaInv.multiply(rap.cross(n)).cross(rap)).add((IbInv.multiply(rbp.cross(n)).cross(rbp)))).dot(n)
			);

			// now work out friction parts
			// direction of friction linear
			Vec3d tal = (n.cross(vabDiff)).cross(n).normalise();

			Vec3d taa = (n.cross(vba1)).cross(n).normalise();

			// angular velocity impulse
			Vec3d la2f = IaInv.multiply(rap.cross((n.scale(jaa).add(taa.scale(FRICTION)))));
			Vec3d lb2f = IbInv.multiply(rbp.cross((n.scale(jba).add(taa.scale(FRICTION)))));

			// velocity impulse
			Vec3d va2 = (n.scale(jal/ma).add(tal.scale(FRICTION/ma)));
			Vec3d vb2 = (n.scale(jal/mb).subtract(tal.scale(FRICTION/mb)));

			// add impulse
			cuboid.addImpulse(rbp.normalise().scale(-collisionDistance/2.0), va2, la2f);
			sphere.addImpulse(rbp.normalise().scale(collisionDistance/2.0), vb2.neg(), lb2f.neg());

		}

	}

	private void sphereCollision(RigidBody rigidBody, RigidBody otherBody) {

		// get radius of both spheres
		// use x dimension for now
		double rigidBodyRadius = rigidBody.getDimensions().getX() / 2.0;
		double otherBodyRadius = otherBody.getDimensions().getX() / 2.0;
		double totalRad = (rigidBodyRadius + otherBodyRadius);

		// get vec between the 2 centers
		Vec3d fromOtherBodyToRigid = rigidBody.getOrigin().subtract(otherBody.getOrigin());
		double length = fromOtherBodyToRigid.length();

		double collisionDistance = length - totalRad;


		// check if collision
		if (collisionDistance < 0) {

			// this is all the maths that will work even if they aren't spheres
			// for spheres, most of it wont do anything

			double ma = rigidBody.getMass();
			double mb = otherBody.getMass();
			// normal of impact point on other body
			Vec3d n = fromOtherBodyToRigid.normalise().neg();

			// distance vec from center of mass of A to point of impact
			Vec3d rap = n.scale(rigidBodyRadius);
			// distance vec from center of mass of B to point of impact
			Vec3d rbp = n.neg().scale(rigidBodyRadius);

			// initial angular velocities
			Vec3d wa1 = rigidBody.getAngularVelocity();
			Vec3d wb1 = otherBody.getAngularVelocity();

			// initial velocities
			Vec3d va1 = rigidBody.getVelocity();
			Vec3d vb1 = otherBody.getVelocity();

			// initial velocity of impact point on A
			Vec3d vap1 = va1.add(wa1.cross(rap));
			// initial velocity of impact point on B
			Vec3d vbp1 = vb1.add(wb1.cross(rbp));

			// initial relative velocity of point on A
			Vec3d vabDiff = va1.subtract(vb1);

			// initial relative velocity of point on B
			Vec3d vba1 = vbp1.subtract(vap1);

			// intertal tensor inverse
			Matrix4d IaInv = rigidBody.getIinv();
			Matrix4d IbInv = otherBody.getIinv();

			// impulse param j
			double jaa = -(1.0+Cr) *  (rigidBody.getVelocity().subtract(otherBody.getVelocity()).dot(n)) / (
					(1.0/ma) +
							(1.0/mb) +
							((IaInv.multiply(rap.cross(n)).cross(rap)).add((IbInv.multiply(rbp.cross(n)).cross(rbp)))).dot(n)
			);
			double jba = -(1.0+Cr) * (otherBody.getVelocity().subtract(rigidBody.getVelocity()).dot(n)) / (
					(1.0/ma) +
							(1.0/mb) +
							((IaInv.multiply(rap.cross(n)).cross(rap)).add((IbInv.multiply(rbp.cross(n)).cross(rbp)))).dot(n)
			);

			// now work out friction parts
			Vec3d taa = (n.cross(vba1)).cross(n).normalise();

			// angular velocity impulse
			Vec3d la2f = IaInv.multiply(rap.cross((n.scale(jaa).add(taa.scale(FRICTION)))));
			Vec3d lb2f = IbInv.multiply(rbp.cross((n.scale(jba).add(taa.scale(FRICTION)))));

			// velocity impulse
			Vec3d va2 = (n.scale(jaa/ma).add(taa.scale(FRICTION/ma)));
			Vec3d vb2 = (n.scale(jba/mb).subtract(taa.scale(FRICTION/mb)));

			// add impulse
			rigidBody.addImpulse(rbp.normalise().scale(-collisionDistance/2.0), va2, la2f);
			otherBody.addImpulse(rbp.normalise().scale(collisionDistance/2.0), vb2, lb2f.neg());

		}
	}

	private void sphereInnerCollision(RigidBody sphereInner, RigidBody sphere) {

		// get radius of both spheres
		// use x dimension for now
		double rigidBodyRadius = sphereInner.getDimensions().getX() / 2.0;
		double otherBodyRadius = sphere.getDimensions().getX() / 2.0;
		double totalRadSqr = (rigidBodyRadius - otherBodyRadius);

		// get vec between the 2 centers
		Vec3d fromOtherBodyToRigid = sphereInner.getOrigin().subtract(sphere.getOrigin());
		double length = fromOtherBodyToRigid.length();

		double collisionDistance = length - totalRadSqr;


		// check if collision
		if (collisionDistance > 0) {

			// this is all the maths that will work even if they aren't spheres
			// for spheres, most of it wont do anything

			double ma = sphereInner.getMass();
			double mb = sphere.getMass();
			// normal of impact point on other body
			Vec3d n = fromOtherBodyToRigid.normalise().neg();

			// distance vec from center of mass of A to point of impact
			Vec3d rap = n.scale(rigidBodyRadius);
			// distance vec from center of mass of B to point of impact
			Vec3d rbp = n.neg().scale(otherBodyRadius);

			// initial angular velocities
			Vec3d wa1 = sphereInner.getAngularVelocity();
			Vec3d wb1 = sphere.getAngularVelocity();

			// initial velocities
			Vec3d va1 = sphereInner.getVelocity();
			Vec3d vb1 = sphere.getVelocity();

			// initial velocity of impact point on A
			Vec3d vap1 = va1.add(wa1.cross(rap));
			// initial velocity of impact point on B
			Vec3d vbp1 = vb1.add(wb1.cross(rbp));

			// initial relative velocity of point on A
			Vec3d vabDiff = va1.subtract(vb1);

			// initial relative velocity of point on B
			Vec3d vba1 = vbp1.subtract(vap1);

			// intertal tensor inverse
			Matrix4d IaInv = sphereInner.getIinv();
			Matrix4d IbInv = sphere.getIinv();


			// impulse param j
			double jaa = -(1.0+Cr) *  (sphereInner.getVelocity().subtract(sphere.getVelocity()).dot(n)) / (
					(1.0/ma) +
							(1.0/mb) +
							((IaInv.multiply(rap.cross(n)).cross(rap)).add((IbInv.multiply(rbp.cross(n)).cross(rbp)))).dot(n)
			);
			double jba = -(1.0+Cr) * (sphere.getVelocity().subtract(sphereInner.getVelocity()).dot(n)) / (
					(1.0/ma) +
							(1.0/mb) +
							((IaInv.multiply(rap.cross(n)).cross(rap)).add((IbInv.multiply(rbp.cross(n)).cross(rbp)))).dot(n)
			);

			// now work out friction parts
			Vec3d taa = (n.cross(vba1)).cross(n).normalise();

			// angular velocity impulse
			Vec3d la2f = IaInv.multiply(rap.cross((n.scale(jaa).add(taa.scale(FRICTION)))));
			Vec3d lb2f = IbInv.multiply(rbp.cross((n.scale(jba).add(taa.scale(FRICTION)))));

			// velocity impulse
			Vec3d va2 = (n.scale(jaa/ma).add(taa.scale(FRICTION/ma)));
			Vec3d vb2 = (n.scale(jba/mb).subtract(taa.scale(FRICTION/mb)));

			// add impulse
			sphereInner.addImpulse(rbp.normalise().scale(-collisionDistance/2.0), va2, la2f);
			sphere.addImpulse(rbp.normalise().scale(collisionDistance/2.0), vb2, lb2f.neg());

		}
	}
}
