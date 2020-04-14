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
	Vector3f _N0, _N1;
	if(slab.N.dot(ray.D) != 0.0f)
	{
		tt0 = -(slab.d0 + slab.N.dot(ray.Q)) / slab.N.dot(ray.D);
		tt1 = -(slab.d1 + slab.N.dot(ray.Q)) / slab.N.dot(ray.D);
		_N0 = -slab.N;
		_N1 = slab.N;
		if(tt0 > tt1)
		{
			float t = tt0;
			tt0 = tt1;
			tt1 = t;
			_N0 = -_N0;
			_N1 = -_N1;
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
		N0 = _N0;
	}
	if (t1 == tt1)
	{
		N1 = _N1;
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
	float t = std::numeric_limits<float>::max();
	if (pT < nT && pT > Epsilon)
	{
		t = pT;
	}
	else if(pT > nT && nT > Epsilon)
	{
		t = nT;
	}
	else if (pT > 0.0f && pT > Epsilon)
	{
		t = pT;
	}
	else if(nT > 0.0f && nT > Epsilon)
	{
		t = nT;
	}
	Vector3f point = ray.Evaluate(t);
	Vector3f normal = (point - center).normalized();
	float theta = atan2f(normal.dot(Vector3f(0.0f, 1.0f, 0.0f)), normal.dot(Vector3f(1.0f, 0.0f, 0.0f)));
	float fi = acos(normal.dot(Vector3f(0.0f, 0.0f, 1.0f)));
	Vector2f uv = Vector2f(theta/(2*PI), fi/PI);
	data.update(t, point, normal, uv, object);
	return true;
}

Bbox Sphere::Bounding_Box() const
{
	Vector3f r = Vector3f(radius, radius, radius);
	return Bbox(center - r, center + r);
}

float Sphere::Distance(const Vector3f& P)
{
	return (P - center).norm() - radius;
}

Box::Box(Vector3f c, Vector3f d) : corner(c), diagonal(d)
{
	slabs[0].N = Vector3f(1, 0, 0);
	slabs[0].d0 = -corner.x();
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
	for (int i = 0; i < 3; ++i)
	{
		interval.Intersect(ray, slabs[i]);
	}

	if (interval.t0 > interval.t1 || (interval.t0 < 0.0f && interval.t1 < 0.0f))
	{
		return false; // No intersection
	}
	float t = std::numeric_limits<float>::max();
	Vector3f normal;
	if (interval.t0 <= interval.t1 && interval.t0 > Epsilon)
	{
		t = interval.t0;
		normal = interval.N0;
	}
	else if (interval.t1 <= interval.t0 && interval.t1 > Epsilon)
	{
		t = interval.t1;
		normal = interval.N1;
	}
	Vector3f point = ray.Evaluate(t);
	Vector2f uv = Vector2f(0, 0);
	data.update(t, point, normal, uv, object);
	if (t == std::numeric_limits<float>::max())
	{
		return false;
	}
	else
	{
		return true;
	}
}

bool Box::IntersectBoundingBox(Ray ray, Intersection& data, Interval& interval)
{
	for (int i = 0; i < 3; ++i)
	{
		interval.Intersect(ray, slabs[i]);
	}

	if (interval.t0 > interval.t1 || (interval.t0 < 0.0f && interval.t1 < 0.0f))
	{
		return false; // No intersection
	}
	float t = std::numeric_limits<float>::max();
	Vector3f normal;
	if (interval.t0 <= interval.t1 && interval.t0 >= 0.0f)
	{
		t = interval.t0;
		normal = interval.N0;
	}
	else if(interval.t1 < interval.t0 && interval.t1 >= 0.0f)
	{
		t = interval.t1;
		normal = interval.N1;
	}
	Vector3f point = ray.Evaluate(t);
	Vector2f uv = Vector2f(0, 0);
	data.update(t, point, normal, uv, object);
	if(t == std::numeric_limits<float>::max())
	{
		return false;		
	}
	else
	{
		return true;
	}
}

Bbox Box::Bounding_Box() const
{
	return Bbox(corner, corner + diagonal);
}

float Box::Distance(const Vector3f& P)
{
	Vector3f& min = corner;
	Vector3f max = corner + diagonal;
	return std::max(std::max(std::max(P.x() - max.x(),min.x() - P.x()),std::max(P.y() - max.y(),min.y() - P.y()))
		,std::max(P.z() - max.z(),min.z() - P.z()));
}

Cylinder::Cylinder(Vector3f b, Vector3f a, float r) : base(b), axis(a), radius(r)
{
	Vector3f rad = Vector3f(radius, radius, radius);
	Vector3f _a = base + rad;
	Vector3f _b = base - rad;
	Vector3f _c = base + axis - rad;
	Vector3f _d = base + axis + rad;
	float minx = std::min(_a.x(), std::min(_b.x(), std::min(_c.x(), _d.x())));
	float miny = std::min(_a.y(), std::min(_b.y(), std::min(_c.y(), _d.y())));
	float minz = std::min(_a.z(), std::min(_b.z(), std::min(_c.z(), _d.z())));

	float maxx = std::max(_a.x(), std::max(_b.x(), std::max(_c.x(), _d.x())));
	float maxy = std::max(_a.y(), std::max(_b.y(), std::max(_c.y(), _d.y())));
	float maxz = std::max(_a.z(), std::max(_b.z(), std::max(_c.z(), _d.z())));

	min = Vector3f(minx, miny, minz);
	max = Vector3f(maxx, maxy, maxz);
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
	float a = newRay.D.x() * newRay.D.x() + newRay.D.y() * newRay.D.y();
	float b = 2 * (newRay.D.x() * newRay.Q.x() + newRay.D.y() * newRay.Q.y());
	float c = newRay.Q.x() * newRay.Q.x() + newRay.Q.y() * newRay.Q.y() - radius * radius;

	float det = b * b - 4 * a * c;
	if(det < 0.0f)
	{
		return false; // No Intersection
	}
	
	t_plus = (-b + sqrt(det)) / (2 * a);
	t_minus = (-b - sqrt(det)) / (2 * a);

	if (interval.t0 < t_minus)
	{
		interval.t0 = t_minus;
	}
	if (interval.t1 > t_plus)
	{
		interval.t1 = t_plus;
	}

	if (interval.t0 > interval.t1 || (interval.t0 < Epsilon && interval.t1 < Epsilon))
	{
		return false; // No Intersection, The "off the corner" case
	}
	float t = std::numeric_limits<float>::max();
	Vector3f t_normal;
	if (interval.t0 < interval.t1 && interval.t0 > Epsilon)
	{
		t = interval.t0;
		t_normal = interval.N0;
	}
	else if(interval.t1 < interval.t0 && interval.t1 > Epsilon)
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
	data.update(t, point, normal, uv,object);
	if (t == std::numeric_limits<float>::max())
	{
		return false;
	}
	else
	{
		return true;
	}
}

Bbox Cylinder::Bounding_Box() const
{
	return Bbox(min, max);
}

float Cylinder::Distance(const Vector3f& P)
{
	return Vector2f(P.x(), P.y()).norm() - radius;
}

Triangle::Triangle(MeshData* meshdata, TriData tridata)
{
	V0 = meshdata->vertices[tridata.x()].pnt;
	N0 = meshdata->vertices[tridata.x()].nrm;
	T0 = meshdata->vertices[tridata.x()].tex;

	V1 = meshdata->vertices[tridata.y()].pnt;
	N1 = meshdata->vertices[tridata.y()].nrm;
	T1 = meshdata->vertices[tridata.y()].tex;

	V2 = meshdata->vertices[tridata.z()].pnt;
	N2 = meshdata->vertices[tridata.z()].nrm;
	T2 = meshdata->vertices[tridata.z()].tex;

	float minx = std::min(V0.x(), std::min(V1.x(), V2.x()));
	float miny = std::min(V0.y(), std::min(V1.y(), V2.y()));
	float minz = std::min(V0.z(), std::min(V1.z(), V2.z()));

	float maxx = std::max(V0.x(), std::max(V1.x(), V2.x()));
	float maxy = std::max(V0.y(), std::max(V1.y(), V2.y()));
	float maxz = std::max(V0.z(), std::max(V1.z(), V2.z()));

	min = Vector3f(minx, miny, minz);
	max = Vector3f(maxx, maxy, maxz);
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
	if (t < Epsilon)
	{
		return false; // No Intersection, Ray's negative half intersects triangle
	}
	Vector3f point = ray.Evaluate(t);
	Vector3f normal = (1 - u - v) * N0 + u*N1 + v*N2;
	Vector2f uv = (1 - u - v) * T0 + u * T1 + v * T2;
	data.update(t, point, normal, uv,object);
	return true;
}

Bbox Triangle::Bounding_Box() const
{
	return Bbox(min, max);
}

float Triangle::Distance(const Vector3f& P)
{
	return 0.0f;
}

float Minimizer::minimumOnObject(Shape* obj)
{
	obj->Intersect(ray,*data);
	return data->t;
}

float Minimizer::minimumOnVolume(const Bbox& box)
{
	Vector3f L = box.min();
	Vector3f U = box.diagonal();

	Box b(L, U);
	Intersection d;
	Interval inter;
	b.IntersectBoundingBox(ray, d, inter);
	if(inter.t0 == 0.0f && inter.t1 == std::numeric_limits<float>::max())
	{
		return 0.0f;
	}
	else
	{
		return d.t;
	}
}

void Intersection::update(float tvalue, Vector3f P, Vector3f N, Vector2f UV, Obj* obj)
{
	if (tvalue < t)
	{
		this->t = tvalue;
		this->P = P;
		this->N = N.normalized();
		this->UV = UV;
		this->object = obj;
	}
}
