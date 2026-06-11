// Small math helpers for composing SteamVR poses.
#pragma once

#include <cmath>

#include <openvr_driver.h>

namespace vrm {

inline vr::HmdVector3d_t
mat_position(const vr::HmdMatrix34_t &m)
{
	return {m.m[0][3], m.m[1][3], m.m[2][3]};
}

inline vr::HmdQuaternion_t
mat_orientation(const vr::HmdMatrix34_t &m)
{
	vr::HmdQuaternion_t q;
	q.w = std::sqrt(std::fmax(0.0, 1.0 + m.m[0][0] + m.m[1][1] + m.m[2][2])) / 2.0;
	q.x = std::sqrt(std::fmax(0.0, 1.0 + m.m[0][0] - m.m[1][1] - m.m[2][2])) / 2.0;
	q.y = std::sqrt(std::fmax(0.0, 1.0 - m.m[0][0] + m.m[1][1] - m.m[2][2])) / 2.0;
	q.z = std::sqrt(std::fmax(0.0, 1.0 - m.m[0][0] - m.m[1][1] + m.m[2][2])) / 2.0;
	q.x = std::copysign(q.x, m.m[2][1] - m.m[1][2]);
	q.y = std::copysign(q.y, m.m[0][2] - m.m[2][0]);
	q.z = std::copysign(q.z, m.m[1][0] - m.m[0][1]);
	return q;
}

// a * b
inline vr::HmdQuaternion_t
operator*(const vr::HmdQuaternion_t &a, const vr::HmdQuaternion_t &b)
{
	return {
	    a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
	    a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
	    a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
	    a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
	};
}

// Rotate a vector by a quaternion.
inline vr::HmdVector3d_t
rotate(const vr::HmdQuaternion_t &q, const vr::HmdVector3d_t &v)
{
	const double tx = 2.0 * (q.y * v.v[2] - q.z * v.v[1]);
	const double ty = 2.0 * (q.z * v.v[0] - q.x * v.v[2]);
	const double tz = 2.0 * (q.x * v.v[1] - q.y * v.v[0]);
	return {
	    v.v[0] + q.w * tx + (q.y * tz - q.z * ty),
	    v.v[1] + q.w * ty + (q.z * tx - q.x * tz),
	    v.v[2] + q.w * tz + (q.x * ty - q.y * tx),
	};
}

// Euler degrees (x, y, z) -> quaternion, Rx*Ry*Rz order.
inline vr::HmdQuaternion_t
euler_deg(float x, float y, float z)
{
	const double hx = x * M_PI / 360.0, hy = y * M_PI / 360.0, hz = z * M_PI / 360.0;
	vr::HmdQuaternion_t qx{std::cos(hx), std::sin(hx), 0, 0};
	vr::HmdQuaternion_t qy{std::cos(hy), 0, std::sin(hy), 0};
	vr::HmdQuaternion_t qz{std::cos(hz), 0, 0, std::sin(hz)};
	return qx * qy * qz;
}

} // namespace vrm
