#pragma once

#include "geom.h"

class Ray
{
public:
	Vector3f Q; // Starting point
	Vector3f D; // Unit length direction of the ray

	Vector3f Evaluate(float t)
	{
		return Q + t * D;
	}
};

class Slab
{
public:
	Vector3f N;		// Normal 
	float d0, d1;
};

class Obj;
class Intersection
{
public:
	float t;		// Parameter value on ray of point of intersection
	Obj* object;	// A pointer to Shape intersected
	Vector3f P;		// Point of intersection (in world coordinates)
	Vector3f N;		// Normal of surface at the intersection point (in world coordinates)
	Vector2f UV;	// Texture coordinates at the intersection point

	void update(float tvalue, Vector3f P, Vector3f N, Vector2f UV);
};

class Interval
{
public:
	float t0, t1;		// Beginning and ending point along a ray
	Vector3f N0, N1;	// Surface normals at t0 and t1 respectively

	Interval();
	Interval(float t0, Vector3f N0, float t1, Vector3f N1);
	void Empty();
	void Intersect(Ray ray, Slab slab);
};

class Shape
{
public:
	virtual bool Intersect(Ray ray, Intersection& data) = 0;
};

class Sphere : public Shape
{
public:
	Vector3f center;
	float radius;

	bool Intersect(Ray ray, Intersection& data) override;
};

class Box : public Shape
{
public:
	Slab slabs[3];
	
	Box(Vector3f corner, Vector3f diagonal);	
	bool Intersect(Ray ray, Intersection& data) override;
};

class Cylinder : public Shape
{
public:
	Vector3f base;
	Vector3f axis;
	float radius;
	
	Cylinder(Vector3f b, Vector3f a, float r);
	bool Intersect(Ray ray, Intersection& data) override;
};

class Triangle : public Shape
{
public:
	bool Intersect(Ray ray, Intersection& data) override;
};