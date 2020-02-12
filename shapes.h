#pragma once

#include <Eigen/StdVector>

using namespace Eigen;
class Ray
{
public:
	Vector3f Q; // Starting point
	Vector3f D; // Unit length direction of the ray
	Ray(Vector3f _q = Vector3f(0.0f, 0.0f, 0.0f), Vector3f _d = Vector3f(0.0f, 0.0f, 0.0f)) : Q(_q), D(_d)
	{
	}
	
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

	Intersection():t(std::numeric_limits<float>::max()),
	object(nullptr),
	P(Vector3f(0.0, 0.0f, 0.0f)),
	N(Vector3f(0.0f, 0.0f, 0.0f)),
	UV(Vector2f(0.0f, 0.0f)){}
	void update(float tvalue, Vector3f P, Vector3f N, Vector2f UV, Obj* obj);
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
	Obj* object;
	
	virtual bool Intersect(Ray ray, Intersection& data) = 0;
};

class Sphere : public Shape
{
public:
	Vector3f center;
	float radius;

	Sphere(Vector3f center, float radius);
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


struct MeshData;
typedef Eigen::Matrix<unsigned int, 3, 1 > TriData;
class Triangle : public Shape
{
public:
	Vector3f V0, V1, V2;
	Vector3f N0, N1, N2;
	Vector2f T0, T1, T2;

	Triangle(MeshData* meshdata,TriData);
	bool Intersect(Ray ray, Intersection& data) override;
};