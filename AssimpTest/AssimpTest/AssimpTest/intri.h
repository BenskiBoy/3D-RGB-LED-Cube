#ifndef INTRI
#define INTRI

#include <iostream>
#include <math.h>
#include <sstream>
#include <fstream>
#include <vector>

#include <mesh.h>
#include <postprocess.h>
#include <cimport.h>
#include <scene.h>
#include <defs.h>
#include <config.h>

#define NO_SHELL (0)
#define INTRI (1)
#define INTRI_TF (2)
#define NOT_INTRI (3)

#define NO_INTERSECTION (0)
#define POSITIVE_INTERSECTION (1)
#define NEGATIVE_INTERSECTION (-1)

const double PI = 3.141592653589793;
// Defines the size and location 
struct rectangle {
	float lox;
	float hix;
	float loy;
	float hiy;
};

rectangle findRectangle(aiVector3D p1, aiVector3D p2, aiVector3D p3);
double intri(aiVector3D* A, aiVector3D* B, aiVector3D* C, aiVector3D* T);
double intri_tf(aiVector3D & A, aiVector3D & At, aiVector3D & Bt, aiVector3D & Ct, aiVector3D & T, aiVector3D & i, aiVector3D & j, aiVector3D & k, double cutoff);
double not_intri(aiVector3D* A, aiVector3D* B, aiVector3D* C, aiVector3D* T);
int volume_test(aiVector3D & A, aiVector3D & At, aiVector3D & Bt, aiVector3D & Ct, aiVector3D & T, aiVector3D & i, aiVector3D & j, aiVector3D & k, aiVector3D & kbar, double cutoff);

#endif