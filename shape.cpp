#include "shape.h"

#define epsilon 0.0001

bool Sphere::intersect(const Ray & ray, Intersection & it)
{
	Vector3f Qbar = ray.Q - center;
	float QdotD = Qbar.dot(ray.D);
	float fvar = sqrt(QdotD*QdotD - Qbar.dot(Qbar) + radius*radius);
	if (-QdotD - fvar > epsilon) it.t = -QdotD - fvar;
	else if (-QdotD + fvar > epsilon) it.t = -QdotD + fvar;
	else return false;

	it.pos = ray.eval(it.t);
	it.normal = (it.pos - center).normalized();
	it.pS = this;

	return true;
}

int Sphere::id = 0;

Box::Box(Vector3f c, Vector3f d, Material * m) : Shape(m), corner(c), diag(d){
	slabs[0] = Slab(Vector3f(1.0f, 0.0f, 0.0f), -c[0], -(c + d)[0]);
	slabs[1] = Slab(Vector3f(0.0f, 1.0f, 0.0f), -c[1], -(c + d)[1]);
	slabs[2] = Slab(Vector3f(0.0f, 0.0f, 1.0f), -c[2], -(c + d)[2]);
	position = corner + diag / 2.0f;
}

bool Box::intersect(const Ray & ray, Intersection & it)
{
	Vector3f normal0, normal1, normalVec;
	float t0 = 0, t1 = FLT_MAX;
	float _t0, _t1;
	for (int i = 0; i < 3; ++i) {
		if (slabs[i].normal.dot(ray.D)) {
			float NdotQ = slabs[i].normal.dot(ray.Q);
			float NdotD = slabs[i].normal.dot(ray.D);
			_t0 = -(slabs[i].d0 + NdotQ) / NdotD;
			_t1 = -(slabs[i].d1 + NdotQ) / NdotD;
			if (_t0 > _t1) {
				float temp = _t0;
				_t0 = _t1;
				_t1 = temp;
			}
			normalVec = slabs[i].normal;
		} else {
			float NdotQ = slabs[i].normal.dot(ray.Q);
			float s0 = NdotQ + slabs[i].d0;
			float s1 = NdotQ + slabs[i].d1;
			if (s0 * s1 < 0.0f) {
				_t0 = 0.0f;
				_t1 = FLT_MAX;
			} else return false;
		}

		if (t0 < _t0) {
			t0 = _t0;
			normal0 = normalVec;
		}
		if (t1 > _t1) {
			t1 = _t1;
			normal1 = normalVec;
		}
	}

	if (t0 > t1) return false;
	else if (t0 > epsilon) {
		it.t = t0; 
		it.normal = normal0;
	}
	else if (t1 > epsilon) {
		it.t = t1;
		it.normal = normal1;
	}
	else return false;

	it.pos = ray.eval(it.t);
	it.normal.normalize();
	Vector3f cornerToIt = it.pos - corner;
	if (cornerToIt[0] < epsilon || cornerToIt[1] < epsilon || cornerToIt[2] < epsilon) {
		if (it.normal.dot(diag) > 0.0f) it.normal *= -1;
	} else if (it.normal.dot(diag) < 0.0f) it.normal *= -1;
	it.pS = this;
	return true;
}

int Cylinder::id = 0;
bool Cylinder::intersect(const Ray & ray, Intersection & it)
{
	Vector3f Q = q._transformVector(ray.Q - base);
	Vector3f D = q._transformVector(ray.D);
	float t0 = 0, t1 = FLT_MAX;
	double b0, b1;
	Vector3f normalVec;
	if (slab.normal.dot(D)) {
		double NdotQ = slab.normal.dot(Q);
		double NdotD = slab.normal.dot(D);
		b0 = -(slab.d0 + NdotQ) / NdotD;
		b1 = -(slab.d1 + NdotQ) / NdotD;
		if (b0 > b1) {
			double temp = b0;
			b0 = b1;
			b1 = temp;
		}
	} else {
		float NdotQ = slab.normal.dot(Q);
		float s0 = NdotQ + slab.d0;
		float s1 = NdotQ + slab.d1;
		if (s0 * s1 < 0.0f) {
			b0 = 0.0f;
			b1 = FLT_MAX;
		} else return false;
	}
	double a = D[0] * D[0] + D[1] * D[1];
	double b = 2 * (D[0] * Q[0] + D[1] * Q[1]);
	double c = Q[0] * Q[0] + Q[1] * Q[1] - radius * radius;
	double deter = b * b - 4.0f * a * c;
	if (deter < 0) return false;
	double c0 = (-b - sqrt(deter)) / 2.0f / a;
	double c1 = (-b + sqrt(deter)) / 2.0f / a;


	Vector3f normal0, normal1, M;
	if (b0 > c0) {
		t0 = b0;
		if (D[2] > epsilon) normal0 = Vector3f(0.0f, 0.0f, -1.0f);
		else normal0 = Vector3f(0.0f, 0.0f, 1.0f);
	} else {
		t0 = c0;
		M = Q + t0*D;
		normal0 = Vector3f(M[0], M[1], 0.0f);
	}

	if (b1 < c1) {
		t1 = b1;
		if (D[2] > epsilon) normal1 = Vector3f(0.0f, 0.0f, 1.0f);
		else normal1 = Vector3f(0.0f, 0.0f, -1.0f);
	} else {
		t1 = c1;
		M = Q + t1*D;
		normal1 = Vector3f(M[0], M[1], 0.0f);
	}

	if (t0 > t1) return false;
	else if (t0 > epsilon) {
		it.t = t0;
		it.normal = q.conjugate()._transformVector(normal0);
	} else if (t1 > epsilon) {
		it.t = t1;
		it.normal = q.conjugate()._transformVector(normal1);
	} else return false;

	it.pos = ray.eval(it.t);
	it.pS = this;
	it.normal.normalize();
	return true;
}

#define epsilon 0.000001
bool Triangle::intersect(const Ray & ray, Intersection & it)
{
	Vector3f e1 = v1 - v0;
	Vector3f e2 = v2 - v0;
	Vector3f p = ray.D.cross(e2);
	float d = p.dot(e1);
	if (abs(d) < epsilon) return false;
	Vector3f s = ray.Q - v0;
	float u = (p.dot(s)) / d;
	if (u < 0.0f || u >1) return false;
	Vector3f q = s.cross(e1);
	float v = ray.D.dot(q) / d;
	if (v < 0.0f || u + v > 1.0f) return false;
	float t = e2.dot(q) / d;
	if (t < 0.0f)return false;

	it.t = t;
	it.pos = ray.eval(t);
	it.normal = e1.cross(e2);
	it.normal.normalize();
	it.text_coord = (1.0f - u - v) * v0_textcord + u * v1_textcord + v * v2_textcord;
	it.pS = this;
	return true;
}

Box3d bounding_box(const Shape *pS) {
	return pS->bbox();
}