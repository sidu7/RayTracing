#include "RayMarch.h"

bool RayMarchShape::Intersect(Ray ray, Intersection& data)
{
	float t = 0.001;
	int steps = 0;
	float DistanceLimit = 10000.0f;
	while (true)
	{
		Vector3f P = ray.Evaluate(t);
		float dt = Distance(P);
		t = t + fabs(dt);
		++steps;
		if (fabs(dt) < 0.000001)
		{
			break;
		}
		if (steps > 2500)
		{
			return false;
		}
		if (t >= DistanceLimit)
		{
			return false;
		}
	}
	Vector3f P = ray.Evaluate(t);
	float h = 0.001;
	float nx = Distance(Vector3f(P.x() + h, P.y(), P.z())) - Distance(Vector3f(P.x() - h, P.y(), P.z()));
	float ny = Distance(Vector3f(P.x(), P.y() + h, P.z())) - Distance(Vector3f(P.x(), P.y() - h, P.z()));
	float nz = Distance(Vector3f(P.x(), P.y(), P.z() + h)) - Distance(Vector3f(P.x(), P.y(), P.z() - h));
	Vector3f N = Vector3f(nx, ny, nz).normalized();
	data.update(t, P, N, Vector2f(0.0f, 0.0f), object);
	return true;
}

float Union::Distance(const Vector3f& P)
{
	return std::min(A->Distance(P), B->Distance(P));
}

float Intersect::Distance(const Vector3f& P)
{
	return std::max(A->Distance(P), B->Distance(P));
}

float Difference::Distance(const Vector3f& P)
{
	return std::max(A->Distance(P), -B->Distance(P));
}

float UniformScale::Distance(const Vector3f& P)
{
	return s * A->Distance(P / s);
}

float NonUniformScale::Distance(const Vector3f& P)
{
	float s_min = std::min(s.x(), std::min(s.y(), s.z()));
	return s_min * A->Distance(Vector3f(P.x()/s.x(),P.y()/s.y(),P.z()/s.z()));
}

Vector3f Twist::Operation(const Vector3f& P)
{
	float c = 1.0 / r;
	float x = P.x() * c * cos(s * P.z()) - P.y() * c * sin(s * P.z());
	float y = P.x() * c * sin(s * P.z()) + P.y() * c * cos(s * P.z());
	float z = P.z();
	return Vector3f(x,y,z);
}

const float PI = 3.14159f;

float Twist::Distance(const Vector3f& P)
{
	float lambda = sqrt(4 + pow(s * PI, 2));
	return A->Distance(Operation(P)) / lambda;
}

Vector3f Taper::Operation(const Vector3f& P)
{
	return Vector3f(0.0f,0.0f,0.0f);
}

float Taper::Distance(const Vector3f& P)
{
	return 0.0f;
}
