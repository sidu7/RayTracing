#pragma once
#include "shapes.h"

class RayMarchShape : public Shape
{
public:
	bool Intersect(Ray ray, Intersection& data) override;
	virtual float Distance(const Vector3f& P) = 0;
	Bbox Bounding_Box() const override { return Bbox(); }
};

class Union : public RayMarchShape
{
public:
	Shape* A;
	Shape* B;
	Union(Shape* _A, Shape* _B) : A(_A), B(_B) {}
	float Distance(const Vector3f& P) override;
};

class Intersect : public RayMarchShape
{
public:
	Shape* A;
	Shape* B;
	Intersect(Shape* _A, Shape* _B) : A(_A), B(_B) {}
	float Distance(const Vector3f& P) override;
};

class Difference : public RayMarchShape
{
public:
	Shape* A;
	Shape* B;
	Difference(Shape* _A, Shape* _B) : A(_A), B(_B) {}
	float Distance(const Vector3f& P) override;
};

class UniformScale : public RayMarchShape
{
public:
	Shape* A;
	float s;
	UniformScale(Shape* _A, float _s) : A(_A), s(_s) {}
	float Distance(const Vector3f& P) override;
};

class NonUniformScale : public RayMarchShape
{
public:
	Shape* A;
	Vector3f s;
	NonUniformScale(Shape* _A, Vector3f _s) : A(_A), s(_s) {}
	float Distance(const Vector3f& P) override;
};

class Taper : public RayMarchShape
{
public:
	Shape* A;
	Vector3f s;
	Taper(Shape* _A, Vector3f _s) : A(_A), s(_s) {}
	Vector3f Operation(const Vector3f& P);
	float Distance(const Vector3f& P) override;
};

class Twist : public RayMarchShape
{
public:
	Shape* A;
	float s;
	float r;
	Twist(Shape* _A, float _s, float _r) : A(_A), s(_s), r(_r) {}
	Vector3f Operation(const Vector3f& P);
	float Distance(const Vector3f& P) override;
};