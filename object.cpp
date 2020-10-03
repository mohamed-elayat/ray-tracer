#include "object.hpp"

#include <cmath>
#include <cfloat>
#include <fstream>
#include <sstream>
#include <map>
#include <vector>
#include <iostream>
#include <algorithm>


bool Object::intersect(Ray ray, Intersection &hit) const 
{
    // Assure une valeur correcte pour la coordonnée W de l'origine et de la direction
	// Vous pouvez commentez ces lignes si vous faites très attention à la façon de construire vos rayons.
    ray.origin[3] = 1;
    ray.direction[3] = 0;

    Ray local_ray(i_transform * ray.origin, i_transform * ray.direction);

    if (localIntersect(local_ray, hit)) 
	{
        // Assure la valeur correcte de W.
        hit.position[3] = 1;
        hit.normal[3] = 0;
        
		// Transforme les coordonnées de l'intersection dans le repère global.
        hit.position = transform * hit.position;
        hit.normal = (n_transform * hit.normal).normalized();
        
		return true;
    }

    return false;
}


bool Sphere::localIntersect(Ray const &ray, Intersection &hit) const 
{

	// coefficients of quadratic equation

    double a =  pow(ray.direction[0], 2) +
                pow(ray.direction[1], 2) +
                pow(ray.direction[2], 2);

    double b =  2 * (ray.origin[0] * ray.direction[0] +
                     ray.origin[1] * ray.direction[1] +
                     ray.origin[2] * ray.direction[2]   );

    double c = pow(ray.origin[0], 2) +
               pow(ray.origin[1], 2) +
               pow(ray.origin[2], 2) -
               pow(this->radius, 2);

    double det = pow(b, 2) - 4 * a * c;

    if(det < 0)
        return false;

    double t1 = (-b + sqrt(det)) / (2 * a);
    double t2 = (-b - sqrt(det)) / (2 * a);
    double tMin = std::min(t1, t2);

    if (tMin < 0 || tMin > hit.depth)
        return false;

    hit.depth = tMin;
    hit.position = ray.origin + ray.direction * tMin;
    hit.normal = hit.position.normalized();

    return true;
//	return false;

}


bool Plane::localIntersect(Ray const &ray, Intersection &hit) const
{

	// if ray starts on plane, and goes along it

    if (ray.origin[2] == 0){
        if (ray.direction[2] == 0) {
            return false;
        }
        else{
            hit.depth = 0;
            hit.position = ray.origin; // + ray.direction * tMin;
//            hit.normal = hit.position.normalized();
            hit.normal = Vector(0.0, 0.0, 1.0);
            return true;
        }
    }

    double t = -1.0 * ray.origin[2] / ray.direction[2];

    if (t > 0 && t < hit.depth){
        hit.depth = t;
        hit.position = ray.origin + ray.direction * t;
//            hit.normal = hit.position.normalized();
        hit.normal = Vector(0.0, 0.0, 1.0);
        return true;
    }
    else{
        return false;
    }

}


bool Conic::localIntersect(Ray const &ray, Intersection &hit) const {
    // @@@@@@ VOTRE CODE ICI (licence créative)
    return false;
}


// Intersections !
bool Mesh::localIntersect(Ray const &ray, Intersection &hit) const
{
	// Test de la boite englobante
	double tNear = -DBL_MAX, tFar = DBL_MAX;
	for (int i = 0; i < 3; i++) {
		if (ray.direction[i] == 0.0) {
			if (ray.origin[i] < bboxMin[i] || ray.origin[i] > bboxMax[i]) {
				// Rayon parallèle à un plan de la boite englobante et en dehors de la boite
				return false;
			}
			// Rayon parallèle à un plan de la boite et dans la boite: on continue
		}
		else {
			double t1 = (bboxMin[i] - ray.origin[i]) / ray.direction[i];
			double t2 = (bboxMax[i] - ray.origin[i]) / ray.direction[i];
			if (t1 > t2) std::swap(t1, t2); // Assure t1 <= t2

			if (t1 > tNear) tNear = t1; // On veut le plus lointain tNear.
			if (t2 < tFar) tFar = t2; // On veut le plus proche tFar.

			if (tNear > tFar) return false; // Le rayon rate la boite englobante.
			if (tFar < 0) return false; // La boite englobante est derrière le rayon.
		}
	}
	// Si on arrive jusqu'ici, c'est que le rayon a intersecté la boite englobante.

	// Le rayon interesecte la boite englobante, donc on teste chaque triangle.
	bool isHit = false;
	for (size_t tri_i = 0; tri_i < triangles.size(); tri_i++) {
		Triangle const &tri = triangles[tri_i];

		if (intersectTriangle(ray, tri, hit)) {
			isHit = true;
		}
	}
	return isHit;
}


//todo: this doesn't calculate C (3rd component of line eq)
double Mesh::implicitLineEquation(double p_x, double p_y,
	double e1_x, double e1_y,
	double e2_x, double e2_y) const
{
	return (e2_y - e1_y)*(p_x - e1_x) - (e2_x - e1_x)*(p_y - e1_y);
}

bool Mesh::intersectTriangle(Ray const &ray,
	Triangle const &tri,
	Intersection &hit) const
{
	// Extrait chaque position de sommet des données du maillage.
	Vector const &p0 = positions[tri[0].pi];
	Vector const &p1 = positions[tri[1].pi];
	Vector const &p2 = positions[tri[2].pi];

    Vector v0 = Vector(p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]);
    Vector v1 = Vector(p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]);
    Vector v2 = Vector(p0[0] - p2[0], p0[1] - p2[1], p0[2] - p2[2]);

    Vector n = v0.cross(v1).normalized();

    if (  ray.direction.normalized().dot(n) > 90 )
        n = -1.0 * n;

    // implicit equation of plane

    double a = n[0];
    double b = n[1];
    double c = n[2];
    double d = -1.0 * n.dot(p2);

    //todo: ray starts on triangle?
    double t = -1.0 * (n.dot(ray.origin) + d) / (n.dot(ray.direction));

    if(t < 0 || t > hit.depth) {
        return false;
    }

    Vector pIn = (p0 + p1 + p2) / 3;

    Vector pTest = Vector(ray.origin[0] + ray.direction[0] * t,
                          ray.origin[1] + ray.direction[1] * t,
                          ray.origin[2] + ray.direction[2] * t);


    v0 = Vector(p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]);
    v1 = Vector(p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]);
//    v2 = Vector(p0[0] - p2[0], p0[1] - p2[1], p0[2] - p2[2]);

    // convert 3D coords of triangle, pIn and pTest into 2D coords
    // on the same plane.

    double p0_2d_x = 0;
    double p0_2d_y = 0;

    double p1_2d_x = v0.length();
    double p1_2d_y = 0;

    double p2_cosTh = v0.normalized().dot(v1.normalized());
    double p2_th = acos(p2_cosTh);
    double p2_sinTh = sin(p2_th); //todo: alternatively, using X product

    double p2_2d_x = v2.length() * p2_cosTh;
    double p2_2d_y = v2.length() * p2_sinTh;

    double pIn_2d_x = (p0_2d_x + p1_2d_x + p2_2d_x) / 3;//    (pIn - p0).length() * pIn_cosTh; // (pIn - p0).length() * p2_cosTh;
    double pIn_2d_y = (p0_2d_y + p1_2d_y + p2_2d_y) / 3; //(pIn - p0).length() * pIn_sinTh;    //v2.length() * p2_sinTh;

    double pTest_cosTh = v0.normalized().dot(  (pTest - p0).normalized()  );
    double pTest_th = acos(pTest_cosTh);
    double pTest_sinTh = sin(pTest_th); //todo: alternatively, using X product

    double pTest_2d_x = (pTest - p0).length() * pTest_cosTh;
    double pTest_2d_y;

    Vector cr1 = v0.cross(  (pIn - p0) );
    Vector cr2 = v0.cross(  (pTest - p0) );

    // if cr1 and cr2, for a given dimension,
    // have different signs, it means that
    // pTest is not on the same side as pIn
    // (i.e. since v0 splits the plane into 2 sides,
    // then pIn and pTest are on different sides, and thus
    // the y component is negative).

    if (  cr1[0] * cr2[0] < 0 ||
            cr1[1] * cr2[1] < 0 ||
            cr1[2] * cr2[2] < 0  ){

        pTest_2d_y = -1.0 * (pTest - p0).length() * pTest_sinTh;
    }
    else{
        pTest_2d_y = (pTest - p0).length() * pTest_sinTh;
    }



    double in1 = this->implicitLineEquation(pIn_2d_x, pIn_2d_y, p0_2d_x, p0_2d_y, p1_2d_x, p1_2d_y);
    double in2 = this->implicitLineEquation(pIn_2d_x, pIn_2d_y, p1_2d_x, p1_2d_y, p2_2d_x, p2_2d_y);
    double in3 = this->implicitLineEquation(pIn_2d_x, pIn_2d_y, p2_2d_x, p2_2d_y, p0_2d_x, p0_2d_y);

    double test1 = this->implicitLineEquation(pTest_2d_x, pTest_2d_y, p0_2d_x, p0_2d_y, p1_2d_x, p1_2d_y);
    double test2 = this->implicitLineEquation(pTest_2d_x, pTest_2d_y, p1_2d_x, p1_2d_y, p2_2d_x, p2_2d_y);
    double test3 = this->implicitLineEquation(pTest_2d_x, pTest_2d_y, p2_2d_x, p2_2d_y, p0_2d_x, p0_2d_y);

    // for any give line between 2 pts on the triangle
    // if pIn and pTest are on different sides, reject.
    // If they're on the same side, they'll have the same sign
    // and thus their * or / will give a positive result.
    // if they're on different sides, they'll have opposing signs
    // and thus their * or / will give a negative result. If
    // the result is negative, reject.

    if(  test1 * in1 < 0 || test2 * in2 < 0 || test3 * in3 < 0 ){
        return false;
    }

    hit.depth = t;
    hit.position = ray.origin + ray.direction * t;
    hit.normal = n.normalized();

    return true;
}
