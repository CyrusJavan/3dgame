#ifndef _BOX_H_
#define _BOX_H_

#include <assert.h>
#include "vector3.h"
#include "ray.h"

/*
 * Axis-aligned bounding box class, for use with the optimized ray-box
 * intersection test described in:
 *
 *      Amy Williams, Steve Barrus, R. Keith Morley, and Peter Shirley
 *      "An Efficient and Robust Ray-Box Intersection Algorithm"
 *      Journal of graphics tools, 10(1):49-54, 2005
 *
 */

class Box {
  public:
    Box() { }
    Box(const Vector3 &min, const Vector3 &max) {
 //     assert(min < max);
      parameters[0] = min;
      parameters[1] = max;
    }
    // (t0, t1) is the interval for valid hits
    bool intersect(const Ray & r, float t0, float t1) const {
        float tmin, tmax, tymin, tymax, tzmin, tzmax;
        if (r.direction.x() >= 0) {
            tmin = (parameters[0].x() - r.origin.x()) / r.direction.x();
            tmax = (parameters[1].x() - r.origin.x()) / r.direction.x();
        }
        else {
            tmin = (parameters[1].x() - r.origin.x()) / r.direction.x();
            tmax = (parameters[0].x() - r.origin.x()) / r.direction.x();
        }
        if (r.direction.y() >= 0) {
            tymin = (parameters[0].y() - r.origin.y()) / r.direction.y();
            tymax = (parameters[1].y() - r.origin.y()) / r.direction.y();
        }
        else {
            tymin = (parameters[1].y() - r.origin.y()) / r.direction.y();
            tymax = (parameters[0].y() - r.origin.y()) / r.direction.y();
        }
        if ( (tmin > tymax) || (tymin > tmax) )
            return false;
        if (tymin > tmin)
            tmin = tymin;
        if (tymax < tmax)
            tmax = tymax;
        if (r.direction.z() >= 0) {
            tzmin = (parameters[0].z() - r.origin.z()) / r.direction.z();
            tzmax = (parameters[1].z() - r.origin.z()) / r.direction.z();
        }
        else {
            tzmin = (parameters[1].z() - r.origin.z()) / r.direction.z();
            tzmax = (parameters[0].z() - r.origin.z()) / r.direction.z();
        }
        if ( (tmin > tzmax) || (tzmin > tmax) )
            return false;
        if (tzmin > tmin)
            tmin = tzmin;
        if (tzmax < tmax)
            tmax = tzmax;
        return ( (tmin < t1) && (tmax > t0) );
    }

    // corners
    Vector3 parameters[2];
	Vector3 min() { return parameters[0]; }
	Vector3 max() { return parameters[1]; }
	const bool inside(const Vector3 &p) {
		return ((p.x() >= parameters[0].x() && p.x() <= parameters[1].x()) &&
		     	(p.y() >= parameters[0].y() && p.y() <= parameters[1].y()) &&
			    (p.z() >= parameters[0].z() && p.z() <= parameters[1].z()));
	}
	const bool inside(Vector3 *points, int size) {
		bool allInside = true;
		for (int i = 0; i < size; i++) {
			if (!inside(points[i])) allInside = false;
			break;
		}
		return allInside;
	}
	Vector3 center() {
		return ((max() - min()) / 2 + min());
	}
};

#endif // _BOX_H_
