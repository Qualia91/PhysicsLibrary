package com.nick.wood.physics;

import com.nick.wood.maths.objects.Quaternion;
import com.nick.wood.maths.objects.vector.Vecd;

import java.util.UUID;

public interface Body {
	UUID getUuid();

	Vecd getOrigin();

	Quaternion getRotation();
}
