#include "shapes.h"
#include "raytrace.h"

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
	slabs[1].N = Vector3f(1, 0, 0);
	slabs[1].d0 = -corner.x;
	slabs[1].d1 = -corner.x - diagonal.x;

	slabs[2].N = Vector3f(0, 1, 0);
	slabs[2].d0 = -corner.y;
	slabs[2].d1 = -corner.y - diagonal.y;

	slabs[3].N = Vector3f(0, 0, 1);
	slabs[3].d0 = -corner.z;
	slabs[3].d1 = -corner.z - diagonal.z;
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
	if (interval.t0 < interval.t1)
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
	return false;
}

bool Triangle::Intersect(Ray ray, Intersection& data)
{
	return false;
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
