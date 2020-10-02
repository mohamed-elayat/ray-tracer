#include <cstdio>
#include <cstdlib>
#include <cfloat>
#include <cmath>
#include <algorithm>
#include <string>
#include <fstream>
#include <vector>
#include <iostream>
#include <sstream>
#include <map>
#include <vector>

#include "raytracer.hpp"
#include "image.hpp"
//#include "basic.hpp"


double zNear;
double zFar;

void Raytracer::render(const char *filename, const char *depth_filename,
                       Scene const &scene)
{
    // Alloue les deux images qui seront sauvegardées à la fin du programme.
    Image colorImage(scene.resolution[0], scene.resolution[1]);
    Image depthImage(scene.resolution[0], scene.resolution[1]);
    
    // Crée le zBuffer.
    double *zBuffer = new double[scene.resolution[0] * scene.resolution[1]];
    for(int i = 0; i < scene.resolution[0] * scene.resolution[1]; i++) {
        zBuffer[i] = DBL_MAX;
    }

	// @@@@@@ VOTRE CODE ICI
	// Calculez les paramètres de la caméra pour les rayons. Référez-vous aux slides pour les détails.
	//!!! NOTE UTILE : tan() prend des radians plutot que des degrés. Utilisez deg2rad() pour la conversion.
	//!!! NOTE UTILE : Le plan de vue peut être n'importe où, mais il sera implémenté différement.
	// Vous trouverez des références dans le cours.

	Vector w = scene.camera.center - scene.camera.position;
    w.normalize();
    w[3] = 0;

    Vector u = w.cross(scene.camera.up);
    u.normalize();
    u[3] = 0;

    Vector v = u.cross(w);
//    v.normalize(); todo: unnecessary as already normed, i think.
    v[3] = 0;

    Matrix camMat = Matrix(u, v, w, scene.camera.position).transpose();
    camMat[3][3] = 1.0;

    double halfHeight = scene.camera.zNear * tan(deg2rad(scene.camera.fovy));
    double halfWidth = halfHeight * scene.camera.aspect;

    Vector O = Vector(-halfWidth, -halfHeight, scene.camera.zNear);

    double pixelWidth = 2 * halfWidth / scene.resolution[0];
    double pixelHeight = 2 * halfHeight / scene.resolution[1];

//    Vector C = Vector(0.0, 0.0, 0.0, 1.0);

    // Itère sur tous les pixels de l'image.
    for(int y = 0; y < scene.resolution[1]; y++) {
        for(int x = 0; x < scene.resolution[0]; x++) {

            // Génère le rayon approprié pour ce pixel.
			Ray ray;
			if (scene.objects.empty())
			{
				// Pas d'objet dans la scène --> on rend la scène par défaut.
				// Pour celle-ci, le plan de vue est à z = 640 avec une largeur et une hauteur toute deux à 640 pixels.
				ray = Ray(scene.camera.position, (Vector(-320, -320, 640) + Vector(x + 0.5, y + 0.5, 0) - scene.camera.position).normalized());
			}
			else
			{
			    Vector xComp = Vector((x + 0.5) * pixelWidth, 0.0, 0.0);
                Vector yComp = Vector(0.0, (y + 0.5) * pixelHeight, 0.0);
			    Vector Pij = O + xComp + yComp;
//                ray = Ray(  scene.camera.position, camMat * Pij  );
                ray = Ray(  scene.camera.position, (camMat * Pij).normalized()  );

				// @@@@@@ VOTRE CODE ICI
				// Mettez en place le rayon primaire en utilisant les paramètres de la caméra.
				//!!! NOTE UTILE : tous les rayons dont les coordonnées sont exprimées dans le
				//                 repère monde doivent avoir une direction normalisée.
				
			}

            // Initialise la profondeur de récursivité du rayon.
            int rayDepth = 0;
           
            // Notre lancer de rayons récursif calculera la couleur et la z-profondeur.
            Vector color;

            // Ceci devrait être la profondeur maximum, correspondant à l'arrière plan.
            // NOTE : Ceci suppose que la direction du rayon est de longueur unitaire (normalisée)
			//        et que l'origine du rayon est à la position de la caméra.
            double depth = scene.camera.zFar;

            // global variables to store zNear and zFar
            zNear = scene.camera.zNear;
            zFar = scene.camera.zFar;

            // Calcule la valeur du pixel en lançant le rayon dans la scène.
            trace(ray, rayDepth, scene, color, depth);

            // Test de profondeur
            if(depth >= scene.camera.zNear && depth <= scene.camera.zFar &&
               depth < zBuffer[x + y*scene.resolution[0]]) {
                zBuffer[x + y*scene.resolution[0]] = depth;

                // Met à jour la couleur de l'image (et sa profondeur)
                colorImage.setPixel(x, y, color);
                depthImage.setPixel(x, y, (depth-scene.camera.zNear) /
                                          (scene.camera.zFar-scene.camera.zNear));
            }


        }

		// Affiche les informations de l'étape
		if (y % 100 == 0)
		{
			printf("Row %d pixels finished.\n", y);
		}
    }

	// Sauvegarde l'image
    colorImage.writeBMP(filename);
    depthImage.writeBMP(depth_filename);

	printf("Ray tracing finished with images saved.\n");

    delete[] zBuffer;
}


bool Raytracer::trace(Ray const &ray,
                       int &rayDepth,
                       Scene const &scene,
                       Vector &outColor, double &depth)
{
    // Incrémente la profondeur du rayon.
    rayDepth++;

    // - itérer sur tous les objets en appelant calling Object::intersect.
    // - ne pas accepter les intersections plus lointaines que la profondeur donnée.
    // - appeler Raytracer::shade avec l'intersection la plus proche.
    // - renvoyer true ssi le rayon intersecte un objet.
    if (scene.objects.empty())
    {
        // Pas d'objet dans la scène --> on rend la scène par défaut :
        // Par défaut, un cube est centré en (0, 0, 1280 + 160) avec une longueur de côté de 320, juste en face de la caméra.
        // Test d'intersection :
        double x = 1280 / ray.direction[2] * ray.direction[0] + ray.origin[0];
        double y = 1280 / ray.direction[2] * ray.direction[1] + ray.origin[1];
        if ((x <= 160) && (x >= -160) && (y <= 160) && (y >= -160))
        {
            // S'il y a intersection :
            Material m; m.emission = Vector(16.0, 0, 0); m.reflect = 0; // seulement pour le matériau par défaut ; vous devrez utiliser le matériau de l'objet intersecté
            Intersection intersection;	// seulement par défaut ; vous devrez passer l'intersection trouvée par l'appel à Object::intersect()
            outColor = shade(ray, rayDepth, intersection, m, scene);
            depth = 1280;	// la profondeur devrait être mise à jour dans la méthode Object::intersect()
        }
    }
    else
    {
        // @@@@@@ VOTRE CODE ICI
        // Notez que pour Object::intersect(), le paramètre hit correspond à celui courant.
        // Votre intersect() devrait être implémenté pour exclure toute intersection plus lointaine que hit.depth

//		Intersection hit;
        Intersection hit = Intersection(); //todo: is this needed? or is the line above enough?
        Material materialOfObjectHit;

        Vector w = scene.camera.center - scene.camera.position;
        w.normalize();
        w[3] = 0;

        Vector u = w.cross(scene.camera.up);
        u.normalize();
        u[3] = 0;

        Vector v = u.cross(w);
        v[3] = 0;

        Matrix camMat = Matrix(u, v, w, scene.camera.position).transpose();

        camMat[3][3] = 1.0;

        Matrix camMatInv = Matrix();
        camMat.invert(camMatInv); // todo: can this ever fail?

        for (auto object : scene.objects){
            if(object->intersect(ray, hit))
                materialOfObjectHit = object->material;
        }

        // ray intersected nothing

        if(hit.depth == DBL_MAX){
            return false;
        }


        // depth test in Z
        // intersection occured further than allowed.
        else if(  (camMatInv * hit.position)[2] > depth ){
            return false;
        }

        else {
            depth =  (camMatInv * hit.position)[2];
            outColor = shade(ray, rayDepth, hit, materialOfObjectHit, scene);
            //rayDepth--; //todo: should we include this line?
            return true;
        }
    }

    // Décrémente la profondeur du rayon.
    //todo: I think the decrementation is unnecessary.
    // when recursion limit is reached, recursion
    // stops and control is recursively returned to
    // the calling functions, all the way to the first
    // trace and then to render.

//    rayDepth--;

    return false;
}

// function used to determine if the shadow ray
// reaches the light source, or if it intersects
// something else.

bool Raytracer::trace2(Ray const &ray,
                      Scene const &scene)
{
    {

//		Intersection hit;
        Intersection hit = Intersection(); //todo: is this needed? or is the line above enough?

        for (auto object : scene.objects){
            if(object->intersect(ray, hit)) {

                // no need to check against zNear or zFar, or any depth.
                // we're strictly concerned about whether something
                // stands in the way of the intersection point and the point light.
                // if there is indeed something in the way, the point is shadowed.

                if(hit.depth > 10e-5){
                    return true;
                }

            }
        }
        return false;
    }

}


Vector Raytracer::shade(Ray const &ray,
                 int &rayDepth,
                 Intersection const &intersection,
                 Material const &material,
                 Scene const &scene)
{
    // - itérer sur toutes les sources de lumières, calculant les contributions ambiant/diffuse/speculaire
    // - utiliser les rayons d'ombre pour déterminer les ombres
    // - intégrer la contribution de chaque lumière
    // - inclure l'émission du matériau de la surface, s'il y a lieu
    // - appeler Raytracer::trace pour les couleurs de reflection/refraction
    // Ne pas réfléchir/réfracter si la profondeur de récursion maximum du rayon a été atteinte !
	//!!! NOTE UTILE : facteur d'atténuation = 1.0 / (a0 + a1 * d + a2 * d * d)..., la lumière ambiante ne s'atténue pas, ni n'est affectée par les ombres
	//!!! NOTE UTILE : n'acceptez pas les intersection des rayons d'ombre qui sont plus loin que la position de la lumière
	//!!! NOTE UTILE : pour chaque type de rayon, i.e. rayon d'ombre, rayon reflechi, et rayon primaire, les profondeurs maximales sont différentes
	Vector diffuse(0);
	Vector ambient(0);
	Vector specular(0);

	ambient = material.ambient;

	for (auto lightIter = scene.lights.begin(); lightIter != scene.lights.end(); lightIter++)
	{
		// @@@@@@ VOTRE CODE ICI
		// Calculez l'illumination locale ici, souvenez-vous d'ajouter les lumières ensemble.
		// Testez également les ombres ici, si un point est dans l'ombre, multipliez ses couleurs diffuse et spéculaire par (1 - material.shadow)

		Vector l = (lightIter->position - intersection.position).normalized();
		Vector n = intersection.normal.normalized();
		Vector r = (-1.0 * l + 2.0 * n.dot(l) * n).normalized();
		Vector v = (-1.0 * intersection.position - scene.camera.position).normalized();

		double d = (lightIter->position - intersection.position).length() + (scene.camera.position - intersection.position).length();
		double attn = 1.0 / (lightIter->attenuation[0] + lightIter->attenuation[1] * d + lightIter->attenuation[2] * d * d);

        Ray ray2 = Ray(intersection.position, l);

        if( trace2(ray2, scene) ){
            diffuse += n.dot(l) * material.diffuse * attn * (1 - material.shadow) * lightIter->diffuse;
            specular += pow( v.dot(r), material.shininess) * material.specular * attn * (1 - material.shadow) * lightIter->specular;
        }
        else{
            diffuse += n.dot(l) * material.diffuse * attn * lightIter->diffuse;
            specular += pow( v.dot(r), material.shininess) * material.specular * attn * lightIter->specular;
        }

	}

	Vector reflectedLight(0);
	if ((!(ABS_FLOAT(material.reflect) < 1e-6)) && (rayDepth < MAX_RAY_RECURSION))
	{
		// @@@@@@ VOTRE CODE ICI
		// Calculez la couleur réfléchie en utilisant trace() de manière récursive.

//        Vector l = (lightIter->position - intersection.position).normalized();
        Vector n = intersection.normal.normalized();
        Vector tmp = ( -1.0 * ray.direction.normalized() );
        Vector r = (-1.0 * tmp + 2.0 * n.dot(tmp) * n).normalized();

        double depthNew = scene.camera.zFar;
        Ray ray3 = Ray(intersection.position, r);

        if(  !trace(ray3, rayDepth, scene, reflectedLight, depthNew)  ){
            reflectedLight = Vector(0);
        }
		
	}

	return material.emission + ambient + diffuse + specular + material.reflect * reflectedLight;
}