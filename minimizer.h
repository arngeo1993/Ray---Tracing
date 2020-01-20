#pragma once
#include "shape.h"

typedef Eigen::AlignedBox<float, 3> Box3d; // The BV type provided by Eigen

class Minimizer
{
public:

	typedef float Scalar; // KdBVH needs Minimizer::Scalar defined
	Ray ray;
	Intersection minIt; // Stuff to track the minimal t and its intersection info

	  // Constructor
	Minimizer(const Ray& r) : ray(r) { minIt.t = FLT_MAX; }
	Minimizer(){}

	Scalar minimumOnObject(Shape* obj);

	Scalar minimumOnVolume(const Box3d& box);
};