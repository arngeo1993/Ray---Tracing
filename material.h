#pragma once

#include <string>
#include "geom.h"
#include "bitmap_image.hpp"
#include "shape.h"
class Material
{
public:
	Vector3f Kd, Ks, Kt;
	float alpha, ior;
	Vector3f color;
	unsigned int texid;
	bool diffuse_bool = false;
	bool specular_bool = false;
	bool normal_bool = false;
	int width, height, n;
	bitmap_image * bimg;
	bitmap_image * bimg_specualr;
	bitmap_image * bimg_normal;
	int diff_spec_norm;
	virtual bool isLight() { return false; }

	Material(): texid(0) {}
	Material(const Vector3f _color): color(_color), texid(0) {}
	Material(const Vector3f d, const Vector3f s, const float a)
		: Kd(d), Ks(s), alpha(a), texid(0) {}
	Material(const Vector3f d, const Vector3f s, const float a, const Vector3f t, float _ior)
		: Kd(d), Ks(s), Kt(t), alpha(a), texid(0), ior(_ior) {}
	Material(const Material &rhs):
		Kd(rhs.Kd), Ks(rhs.Ks), Kt(rhs.Kt), ior(rhs.ior), alpha(rhs.alpha), color(rhs.color), texid(rhs.texid){}
	
	void UV_Tex_Coord(int x, int y, unsigned char * color);
	Vector3f Tex_Pixel(float x, float y);
	void setTexture_diffuse(const std::string path);
	//void setTexture_specular(const std::string path);
	//void setTexture_normal(const std::string path);
	Vector3f getColor(Vector2f uv_point);
	void setTexture(const std::string path);
	//virtual void apply(const unsigned int program);
};

class Light : public Material
{
public:
	Light(const Vector3f _color) :Material(_color) {}

	virtual bool isLight() { return true; }
	//virtual void apply(const unsigned int program);
};

class BRDF : public Material
{
public:
	BRDF(const Vector3f d, const Vector3f s, const float a):Material(d,s,a) {}
	BRDF(const Vector3f d, const Vector3f s, const float a, const Vector3f t, const float ior) :Material(d, s, a, t, ior) {}

	virtual bool isLight() { return false; }
	//virtual void apply(const unsigned int program);
};