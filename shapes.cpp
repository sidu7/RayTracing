#include "shapes.h"
#include "raytrace.h"

#include <Eigen/Geometry>

using namespace Eigen;

Interval::Interval() : t0(0.0f), t1(std::numeric_limits<float>::max())
{
}

Interval::Interval(float t0, Vector3f N0, float t1, Vector3f N1)
{
	if (t0 <= t1)
	{
		this->t0 = t0;
		this->t1 = t1;
		this->N0 = N0;
		this->N1 = N1;
	}
	else
	{
		this->t0 = t1;
		this->t1 = t0;
		this->N0 = N1;
		this->N1 = N0;
	}
}

void Interval::Empty()
{
	t0 = 0.0f;
	t1 = -1.0f;
}

void Interval::Intersect(Ray ray, Slab slab)
{
	float tt0, tt1;
	if(slab.N.dot(ray.D) != 0.0f)
	{
		tt0 = -(slab.d0 + slab.N.dot(ray.Q)) / slab.N.dot(ray.D);
		tt1 = -(slab.d1 + slab.N.dot(ray.Q)) / slab.N.dot(ray.D);
		if(tt0 > tt1)
		{
			float t = tt0;
			tt0 = tt1;
			tt1 = t;
		}
	}
	else
	{
		float s0 = slab.N.dot(ray.Q) + slab.d0;
		float s1 = slab.N.dot(ray.Q) + slab.d1;
		if(std::signbit(s0) != std::signbit(s1))
		{
			tt0 = 0;
			tt1 = std::numeric_limits<float>::max();
		}
		else
		{
			tt0 = 1;
			tt1 = 0;
		}
	}
	
	t0 = std::max(t0, tt0);
	t1 = std::min(t1, tt1);
	if (t0 == tt0)
	{
		N0 = -slab.N;
	}
	if (t1 == tt1)
	{
		N1 = slab.N;
	}
}

Sphere::Sphere(Vector3f c, float r) : center(c), radius(r)
{
}

bool Sphere::Intersect(Ray ray, Intersection& data)
{
	Vector3f Qbar = ray.Q - center;
	float QbarD = Qbar.dot(ray.D);
	float QbarQ = Qbar.dot(Qbar);
	float descriminat = QbarD * QbarD - QbarQ + radius * radius;
	if (descriminat < 0.0f)
	{
		return false; // No intersection
	}
	float sqrval = std::sqrt(descriminat);

	float pT = -QbarD + sqrval;
	float nT = -QbarD - sqrval;

	if (pT < 0.0f && nT < 0.0f)
	{
		return false; // No intersection
	}
	float t;
	if (pT < nT)
	{
		t = pT;
	}
	else
	{
		t = nT;
	}
	Vector3f point = ray.Evaluate(t);
	Vector3f normal = (point - center).normalized();
	float theta = atan2f(normal.dot(Vector3f(0.0f, 1.0f, 0.0f)), normal.dot(Vector3f(1.0f, 0.0f, 0.0f)));
	float fi = acos(normal.dot(Vector3f(0.0f, 0.0f, 1.0f)));
	Vector2f uv = Vector2f(theta/(2*PI), fi/PI);
	data.update(t, point, normal, uv);
	return true;
}

Box::Box(Vector3f corner, Vector3f diagonal)
{
	slabs[0].N = Vector3f(1, 0, 0);
	slabs[0].d0 = corner.x();
	slabs[0].d1 = -corner.x() - diagonal.x();

	slabs[1].N = Vector3f(0, 1, 0);
	slabs[1].d0 = -corner.y();
	slabs[1].d1 = -corner.y() - diagonal.y();

	slabs[2].N = Vector3f(0, 0, 1);
	slabs[2].d0 = -corner.z();
	slabs[2].d1 = -corner.z() - diagonal.z();
}

bool Box::Intersect(Ray ray, Intersection& data)
{
	Interval interval;
	for(int i = 0; i < 3; ++i)
	{
		interval.Intersect(ray, slabs[i]);
	}
	
	if (interval.t0 > interval.t1 || (interval.t0 < 0.0f && interval.t1 < 0.0f))
	{
		return false; // No intersection
	}
	float t;
	Vector3f normal;
	if (interval.t0 < interval.t1 && interval.t0 > 0.0f)
	{
		t = interval.t0;
		normal = interval.N0;
	}
	else
	{
		t = interval.t1;
		normal = interval.N1;
	}
	Vector3f point = ray.Evaluate(t);	
	Vector2f uv = Vector2f(0,0);
	data.update(t, point, normal, uv);
	return true;
}

Cylinder::Cylinder(Vector3f b, Vector3f a, float r) : base(b), axis(a), radius(r)
{
}

bool Cylinder::Intersect(Ray ray, Intersection& data)
{
	Quaternionf q = Quaternionf::FromTwoVectors(axis, Vector3f::UnitZ());
	Ray newRay;
	newRay.Q = q._transformVector(ray.Q - base);
	newRay.D = q._transformVector(ray.D);

	Interval interval;
	Slab s;
	s.N = Vector3f(0, 0, 1);
	s.d0 = 0;
	s.d1 = - axis.norm();
	interval.Intersect(newRay, s);
	float t_minus,t_plus;
	float a = newRay.D.x() * newRay.D.x() + newRay.D.y() + newRay.D.y();
	float b = 2 * (newRay.D.x() * newRay.Q.x() + newRay.D.y() * newRay.Q.y());
	float c = newRay.Q.x() * newRay.Q.x() + newRay.Q.y() * newRay.Q.y() - radius * radius;

	t_plus = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
	t_minus = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);

	if (interval.t0 < t_minus)
	{
		interval.t0 = t_minus;
	}
	if (interval.t1 > t_plus)
	{
		interval.t1 = t_plus;
	}

	if (interval.t0 > interval.t1 || (interval.t0 < 0.0f && interval.t1 < 0.0f))
	{
		return false; // No Intersection, The "off the corner" case
	}
	float t;
	Vector3f t_normal;
	if (interval.t0 < interval.t1 && interval.t0 > 0.0f)
	{
		t = interval.t0;
		t_normal = interval.N0;
	}
	else
	{
		t = interval.t1;
		t_normal = interval.N1;
	}
	Vector3f new_point = newRay.Evaluate(t);
	if (t == t_minus || t == t_plus)
	{
		t_normal = Vector3f(new_point.x(), new_point.y(), 0.0f);
	}
	Vector3f point = ray.Evaluate(t);
	Vector3f normal = q.conjugate()._transformVector(t_normal);
	float theta = atan2(t_normal.y(), t_normal.x());
	Vector2f uv = Vector2f(theta / (2 * PI), t_normal.z() / axis.norm());
	data.update(t, point, normal, uv);
	return true;
}

Triangle::Triangle(MeshData* meshdata)
{
	V0 = meshdata->vertices[0].pnt;
	N0 = meshdata->vertices[0].nrm;
	T0 = meshdata->vertices[0].tex;

	V1 = meshdata->vertices[1].pnt;
	N1 = meshdata->vertices[1].nrm;
	T1 = meshdata->vertices[1].tex;

	V2 = meshdata->vertices[2].pnt;
	N2 = meshdata->vertices[2].nrm;
	T2 = meshdata->vertices[2].tex;
}

bool Triangle::Intersect(Ray ray, Intersection& data)
{
	Vector3f E1 = V1 - V0;
	Vector3f E2 = V2 - V0;

	Vector3f p = ray.D.cross(E2);
	float d = p.dot(E1);
	
	if (d == 0)
	{
		return false; // No Intersection
	}

	Vector3f S = ray.Q - V0;
	float u = p.dot(S) / d;

	if (u < 0.0f || u > 1.0f)
	{
		return false; // No Intersection, Ray intersects plane, but outside E2 edge 
	}
	Vector3f q = S.cross(E1);

	float v = ray.D.dot(q) / d;

	if (v < 0.0f || (u + v) > 1.0f)
	{
		return false; // No Intersection, Ray intersects plane, but outside other edges
	}

	float t = E2.dot(q) / d;
	if (t < 0.0f)
	{
		return false; // No Intersection, Ray's negative half intersects triangle
	}
	Vector3f point = ray.Evaluate(t);
	Vector3f normal = (1 - u - v) * N0 + u*N1 + v*N2;
	Vector2f uv = (1 - u - v) * T0 + u * T1 + v * T2;
	data.update(t, point, normal, uv);
	return true;
}

void Intersection::update(float tvalue, Vector3f P, Vector3f N, Vector2f UV)
{
	if (tvalue < t)
	{
		this->t = tvalue;
		this->P = P;
		this->N = N;
		this->UV = UV;
	}
}
