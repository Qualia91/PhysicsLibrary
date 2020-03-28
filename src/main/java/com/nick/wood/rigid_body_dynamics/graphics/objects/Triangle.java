package com.nick.wood.rigid_body_dynamics.graphics.objects;

import com.nick.wood.rigid_body_dynamics.graphics.Mesh;
import com.nick.wood.rigid_body_dynamics.graphics.Vertex;
import com.nick.wood.rigid_body_dynamics.maths.Vec3d;

import java.util.ArrayList;

public class Triangle implements MeshObject {

	private static final int TRIANGLE_NUMBER = 20;

	private Mesh mesh;

	public Triangle() {

		ArrayList<ArrayList<Vertex>> vertexSlicesArrayList = new ArrayList<>();
		ArrayList<Integer> indexList = new ArrayList<>();

		Vec3d frontLeft = new Vec3d(-1.0, 1.0, 0.0);
		Vec3d frontRight = new Vec3d(-1.0, -1.0, 0.0);
		Vec3d top = new Vec3d(0.0, 0.0, 1.0);

		double lengthOfTriangleSide =  top.subtract(frontLeft).length() / TRIANGLE_NUMBER;

		Vec3d frontLeftToTopVec = top.subtract(frontLeft).normalise().scale(lengthOfTriangleSide);
		Vec3d topToFrontRightVec = top.subtract(frontRight).normalise().scale(lengthOfTriangleSide).neg();

		// work out vertices in strips
		for (int triangleUpIndex = 0; triangleUpIndex < TRIANGLE_NUMBER + 1; triangleUpIndex++) {

			ArrayList<Vertex> vertexArrayList = new ArrayList<>();

			Vec3d startingPos = frontLeft.add(frontLeftToTopVec.scale(triangleUpIndex));

			// for every triangle side going back down, so max is current triangleUpIndex inclusive
			for (int triangleDownIndex = 0; triangleDownIndex <= triangleUpIndex; triangleDownIndex++) {

				Vec3d newPos = startingPos.add(topToFrontRightVec.scale(triangleDownIndex));

				vertexArrayList.add(new Vertex(
						newPos.normalise(),
						newPos
				));

			}

			vertexSlicesArrayList.add(vertexArrayList);

		}

		// create the 2 returning arrays
		int numOfVerts = ((TRIANGLE_NUMBER + 1) * (TRIANGLE_NUMBER + 1) + (TRIANGLE_NUMBER + 1)) / 2;
		Vertex[] vertices = new Vertex[numOfVerts];

		// use strips to work out indices for triangles
		// start with current strip and only go up to second to last strip
		for (int sliceIndex = 0; sliceIndex < vertexSlicesArrayList.size() - 1; sliceIndex++) {

			ArrayList<Vertex> startingVertexArray = vertexSlicesArrayList.get(sliceIndex);

			// for each vertex in previous slice, get the 2 vertices it links to in the current slice
			for (int i = 0; i < startingVertexArray.size(); i++) {
				int startIndex = ((sliceIndex * sliceIndex) + sliceIndex) / 2;
				vertices[startIndex + i] = startingVertexArray.get(i);

				// get the next strips vertex array
				ArrayList<Vertex> nextVertexArray = vertexSlicesArrayList.get(sliceIndex + 1);

				int nextSliceIndex = sliceIndex + 1;
				int nextStartIndex = ((nextSliceIndex * nextSliceIndex) + nextSliceIndex) / 2;
				int nextIndex = nextStartIndex + i;
				int nextNextIndex = nextStartIndex + i + 1;
				// get vertex in the next strip equal to the current index of vertex, and +1 to this index
				vertices[nextIndex] = nextVertexArray.get(i);
				vertices[nextNextIndex] = nextVertexArray.get(i + 1);

				// add these indexes onto the index array
				indexList.add(startIndex + i);
				indexList.add(nextIndex);
				indexList.add(nextNextIndex);
			}

			// now do the ones i missed. must be a better way of doing this...
			if (sliceIndex != 0) {
				for (int i = 0; i < startingVertexArray.size() - 1; i++) {
					int startIndex = i + ((sliceIndex * sliceIndex) + sliceIndex) / 2;
					int nextStartIndex = startIndex + 1;

					int nextSliceIndex = sliceIndex + 1;
					int nextSliceVertexIndex = (((nextSliceIndex * nextSliceIndex) + nextSliceIndex) / 2) + 1 + i;

					// add these indexes onto the index array
					indexList.add(startIndex);
					indexList.add(nextSliceVertexIndex);
					indexList.add(nextStartIndex);
				}
			}

		}

		int[] indices = new int[indexList.size()];
		for (int i = 0; i < indexList.size(); i++) {
			indices[i] = indexList.get(i);
		}

		mesh = new Mesh(vertices, indices);
	}

	public Mesh getMesh() {
		return mesh;
	}
}
