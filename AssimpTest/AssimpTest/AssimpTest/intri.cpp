#include "intri.h"

rectangle findRectangle(aiVector3D p1, aiVector3D p2, aiVector3D p3) {
	// Find min and max X&Y for face in XY plane
	rectangle rect;
	// horrible if/else chains
	if (p1.x < p2.x) {
		if (p1.x < p3.x) {
			rect.lox = p1.x;
			if (p2.x > p3.x) rect.hix = p2.x; else rect.hix = p3.x;
		}
		else { rect.lox = p3.x; rect.hix = p2.x; }
	}
	else {
		if (p2.x < p3.x) {
			rect.lox = p2.x;
			if (p1.x > p3.x) rect.hix = p1.x;	else rect.hix = p3.x;
		}
		else { rect.lox = p3.x;	rect.hix = p1.x; }
	}
	if (p1.y < p2.y) {
		if (p1.y < p3.y) {
			rect.loy = p1.y;
			if (p2.y > p3.y) rect.hiy = p2.y; else rect.hiy = p3.y;
		}
		else { rect.loy = p3.y; rect.hiy = p2.y; }
	}
	else {
		if (p2.y < p3.y) {
			rect.loy = p2.y;
			if (p1.y > p3.y) rect.hiy = p1.y;	else rect.hiy = p3.y;
		}
		else { rect.loy = p3.y;	rect.hiy = p1.y; }
	}
	return rect;
}

// Checks if T is inside the triangle ABC, Where A, B, C & T are 3D points (aiVector3D)
// Returns the error of T being on the face
double intri(aiVector3D * A, aiVector3D * B, aiVector3D * C, aiVector3D * T) {
	double ATB = (180.0 / PI) * acos(((T->x - A->x)*(T->x - B->x) + (T->y - A->y)*(T->y - B->y) + (T->z - A->z)*(T->z - B->z)) / (sqrt(pow((T->x - A->x), 2) + pow((T->y - A->y), 2) + pow((T->z - A->z), 2)) * sqrt(pow((T->x - B->x), 2) + pow((T->y - B->y), 2) + pow((T->z - B->z), 2))));
	double BTC = (180.0 / PI) * acos(((T->x - B->x)*(T->x - C->x) + (T->y - B->y)*(T->y - C->y) + (T->z - B->z)*(T->z - C->z)) / (sqrt(pow((T->x - B->x), 2) + pow((T->y - B->y), 2) + pow((T->z - B->z), 2)) * sqrt(pow((T->x - C->x), 2) + pow((T->y - C->y), 2) + pow((T->z - C->z), 2))));
	double CTA = (180.0 / PI) * acos(((T->x - C->x)*(T->x - A->x) + (T->y - C->y)*(T->y - A->y) + (T->z - C->z)*(T->z - A->z)) / (sqrt(pow((T->x - C->x), 2) + pow((T->y - C->y), 2) + pow((T->z - C->z), 2)) * sqrt(pow((T->x - A->x), 2) + pow((T->y - A->y), 2) + pow((T->z - A->z), 2))));
	if (ATB > 180.0 || BTC > 180.0 || CTA > 180.0)
		return -1;
	else
		return abs(ATB + BTC + CTA - 360.0);
}

double intri_tf(aiVector3D & A, aiVector3D & At, aiVector3D & Bt, aiVector3D & Ct, aiVector3D & T, aiVector3D & i, aiVector3D & j, aiVector3D & k, double cutoff) {
	double dist;			// LED distance from face
	aiVector3D Tt;	// LED point transposed
	float ACdoti;
	float magAB;
	float magAC;

	Tt.x = i.x*(T.x - A.x) + i.y*(T.y - A.y) + i.z*(T.z - A.z);
	Tt.y = j.x*(T.x - A.x) + j.y*(T.y - A.y) + j.z*(T.z - A.z);
	Tt.z = k.x*(T.x - A.x) + k.y*(T.y - A.y) + k.z*(T.z - A.z);

	// Intri test
	double cATB = ((Tt.x - At.x)*(Tt.x - Bt.x) + (Tt.y - At.y)*(Tt.y - Bt.y)) / (sqrt(pow((Tt.x - At.x), 2) + pow((Tt.y - At.y), 2)) * sqrt(pow((Tt.x - Bt.x), 2) + pow((Tt.y - Bt.y), 2)));
	double cBTC = ((Tt.x - Bt.x)*(Tt.x - Ct.x) + (Tt.y - Bt.y)*(Tt.y - Ct.y)) / (sqrt(pow((Tt.x - Bt.x), 2) + pow((Tt.y - Bt.y), 2)) * sqrt(pow((Tt.x - Ct.x), 2) + pow((Tt.y - Ct.y), 2)));
	double cCTA = ((Tt.x - Ct.x)*(Tt.x - At.x) + (Tt.y - Ct.y)*(Tt.y - At.y)) / (sqrt(pow((Tt.x - Ct.x), 2) + pow((Tt.y - Ct.y), 2)) * sqrt(pow((Tt.x - At.x), 2) + pow((Tt.y - At.y), 2)));
	if (cATB > 1.0) cATB = 1.0;
	else if (cATB < -1.0) cATB = -1.0;
	if (cBTC > 1.0) cBTC = 1.0;
	else if (cBTC < -1.0) cBTC = -1.0;
	if (cCTA > 1.0) cCTA = 1.0;
	else if (cCTA < -1.0) cCTA = -1.0;
	double ATB = (180.0 / PI) * acos(cATB);
	double BTC = (180.0 / PI) * acos(cBTC);
	double CTA = (180.0 / PI) * acos(cCTA);

	if (abs(ATB + BTC + CTA - 360) < cutoff) {
		dist = abs(Tt.z);
	}
	else {
		dist = -1;
	}
	return dist;
}

double not_intri(aiVector3D* A, aiVector3D* B, aiVector3D* C, aiVector3D* T) {
	double Trix = (A->x + B->x + C->x) / 3.0;
	double Triy = (A->y + B->y + C->y) / 3.0;
	double Triz = (A->z + B->z + C->z) / 3.0;
	return sqrt(pow(Trix - T->x, 2) + pow(Triy - T->y, 2) + pow(Triz - T->z, 2));
}

int volume_test(aiVector3D & A, aiVector3D & At, aiVector3D & Bt, aiVector3D & Ct, aiVector3D & T, aiVector3D & i, aiVector3D & j, aiVector3D & k, aiVector3D & kbar, double cutoff) {
	aiVector3D Tt;	// LED point transposed
	float faceX, faceY; // The face co-ordinates where the z line through the LED intersects the face
	float scale; // The scale of k' to reach XY plane from LED
				 // Transfrom LED
	Tt.x = i.x*(T.x - A.x) + i.y*(T.y - A.y) + i.z*(T.z - A.z);
	Tt.y = j.x*(T.x - A.x) + j.y*(T.y - A.y) + j.z*(T.z - A.z);
	Tt.z = k.x*(T.x - A.x) + k.y*(T.y - A.y) + k.z*(T.z - A.z);
	// Find face intersection
	scale = Tt.z / kbar.z;
	faceX = Tt.x - scale*kbar.x;
	faceY = Tt.y - scale*kbar.y;
	// Intri test
	double ATB = acos(((faceX - At.x)*(faceX - Bt.x) + (faceY - At.y)*(faceY - Bt.y)) / (sqrt(pow((faceX - At.x), 2) + pow((faceY - At.y), 2)) * sqrt(pow((faceX - Bt.x), 2) + pow((faceY - Bt.y), 2))));
	double BTC = acos(((faceX - Bt.x)*(faceX - Ct.x) + (faceY - Bt.y)*(faceY - Ct.y)) / (sqrt(pow((faceX - Bt.x), 2) + pow((faceY - Bt.y), 2)) * sqrt(pow((faceX - Ct.x), 2) + pow((faceY - Ct.y), 2))));
	double CTA = acos(((faceX - Ct.x)*(faceX - At.x) + (faceY - Ct.y)*(faceY - At.y)) / (sqrt(pow((faceX - Ct.x), 2) + pow((faceY - Ct.y), 2)) * sqrt(pow((faceX - At.x), 2) + pow((faceY - At.y), 2))));
	if (abs(ATB + BTC + CTA - 2 * PI) < cutoff) {
		if (scale > 0.0) return NEGATIVE_INTERSECTION;
		else return POSITIVE_INTERSECTION;
	}
	return NO_INTERSECTION;
}