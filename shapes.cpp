#include "shapes.h"

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
	Vector3f point, normal;
	Vector2f uv;
	if (pT < nT)
	{
		t = pT;
	}
	else
	{
		t = nT;
	}
	point = ray.Evaluate(t);
	normal = (point - center).normalized();
	float theta = atan2f(normal.dot(Vector3f(0.0f, 1.0f, 0.0f)), normal.dot(Vector3f(1.0f, 0.0f, 0.0f)));
	float fi = acos(normal.dot(Vector3f(0.0f, 0.0f, 1.0f)));
	uv = Vector2f(theta, fi);
	data.update(t, point, normal, uv);
}

bool Box::Intersect(Ray ray, Intersection& data)
{
	return false;
}

bool Cylinder::Intersect(Ray ray, Intersection& data)
{
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
