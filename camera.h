#pragma once

#include "geom.h"

struct Camera {
public:
	Vector3f eye;
	Quaternionf orient;
	float ry;
	float t0;
	float t1;
	float inFocus = 7.0f;
	float diskWidth = 0.4f;
};