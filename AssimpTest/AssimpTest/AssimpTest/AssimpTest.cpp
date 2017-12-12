// AssimpTest.cpp : Defines the entry point for the console application.
//

#include <iostream>
#include "math.h"
#include <sstream>
#include <fstream>
#include <vector>
#include <thread>
#include <mutex>

#include "mesh.h"
#include "postprocess.h"
#include "cimport.h"
#include "scene.h"
#include "defs.h"
#include "config.h"
#include "texture.h"

#include "SDL.h" 
#include "SDL_image.h"
#include <iomanip>
#include "IL\il.h"

#include "intri.h"
#include "utils.h"

//using namespace cimg_library;
using namespace std;
using namespace Assimp;

// Profiling tools
#define PROFILE 0
#if PROFILE
#define CLOCK_DEF(c) clock_t (CLOCK_##c) = 0;
CLOCK_DEF(Total);
CLOCK_DEF(Setup);
CLOCK_DEF(MeshPre);
CLOCK_DEF(FacePre);
CLOCK_DEF(FaceBoundTest);
CLOCK_DEF(LedPre);
CLOCK_DEF(Face_Transform);
CLOCK_DEF(Volume);
CLOCK_DEF(Intri);
CLOCK_DEF(Intensity);
CLOCK_DEF(Colour);
CLOCK_DEF(Display);
CLOCK_DEF(IntensityAndVolume);
CLOCK_DEF(Loop);

#define CLOCK_START(c) (CLOCK_##c) -= clock()
#define CLOCK_STOP(c)  (CLOCK_##c) += clock()
#define CLOCK_STOPSTART(c1,c2) {\
		clock_t CLOCK_TMP = clock();\
		(CLOCK_##c1) += CLOCK_TMP;\
		(CLOCK_##c2) -= CLOCK_TMP;\
	}
#define CLOCK_DISP(c)    cout <<"Clock "<< setw(20) << #c <<": "<<setw(7)<<fixed<<setprecision(3)<< double(CLOCK_##c)/CLOCKS_PER_SEC << " s." << endl; (CLOCK_##c) = 0;
#define CLOCK_DISPCUM(c) cout <<"Clock "<< setw(20) << #c <<": "<<setw(7)<<fixed<<setprecision(3)<< double(CLOCK_##c)/CLOCKS_PER_SEC << " s (cumulative)." << endl;
#define CLOCK_DISP_RUNNING(c) {\
		clock_t CLOCK_TMP = clock();\
		(CLOCK_##c) += CLOCK_TMP;\
		CLOCK_DISP(c);\
		(CLOCK_##c) -= CLOCK_TMP;\
	}
#define CLOCK_DISPCUM_RUNNING(c) {\
		clock_t CLOCK_TMP = clock();\
		(CLOCK_##c) += CLOCK_TMP;\
		CLOCK_DISPCUM(c);\
		(CLOCK_##c) -= CLOCK_TMP;\
	}
#else
#define CLOCK_START(c)
#define CLOCK_STOP(c)
#define CLOCK_STOPSTART(c1,c2)
#define CLOCK_DISP(c)
#define CLOCK_DISPCUM(c)
#define CLOCK_DISP_RUNNING(c)
#define CLOCK_DISPCUM_RUNNING(c)
#endif // PROFILE

#define FOM (aiColor3D(1.0f,0.0f,1.0f))

// Conversion parameters
std::string fname = "simpleman/normal_man";
std::string mname = "simpleman/simpleMan2.6.obj";
aiColor3D * lit = new aiColor3D(0.0, 0.8, 0.6);	// Shell colour
aiColor3D * vlit = new aiColor3D(0.4, 0.0, 0.0);	// Fill colour
aiColor3D * unlit = new aiColor3D(0.0, 0.0, 0.0);  // Background colour
float decay = 5000.0;								// Rate of brightness decay for distance from surface
double intri_cutoff = 0.01;							// Maximum error to be considered inside a triangle
double intri_cutoff_volume = 0.1;					// Maximum error to be considered inside a triangle (for volume filling)
bool volume = false;								// Determines if volume is filled
int shell_type = INTRI_TF;							// Surface lighting algorithm
double max_brightness = 0.8;						// LED scaling factor
bool silent = true;									// Will not output info by default
bool progress = true;								// Displays progress bar, can be disabled
float e_cutoff = log(100) / log(::decay);
unsigned int nthreads = 8;

void colour_face(vector<aiVector3D> & locations, aiScene * scene, const int minMax, const vector<aiColor3D> materialColors, const vector<aiString> texturePath, const vector<SDL_Surface*> image) {
	progress_index do_face;
	while (true) {
		do_face = processing_progress::next_face();
		if (do_face.mesh_index == -1) break;
		// Do face!
		CLOCK_STOPSTART(Loop, FacePre);
		const  aiMesh * m = scene->mMeshes[do_face.mesh_index];
		const aiFace f = m->mFaces[do_face.face_index];
		const unsigned int
			f0 = f.mIndices[0],
			f1 = f.mIndices[1],
			f2 = f.mIndices[2];
		aiVector3D
			a = m->mVertices[f0],
			b = m->mVertices[f1],
			c = m->mVertices[f2];

		CLOCK_STOPSTART(FacePre, FaceBoundTest);

		// Bounding box
		float
			bbx = min(min(a.x, b.x), c.x) - e_cutoff, // Bottom Bound X
			bby = min(min(a.y, b.y), c.y) - e_cutoff,
			bbz = min(min(a.z, b.z), c.z) - e_cutoff,
			tbx = max(max(a.x, b.x), c.x) + e_cutoff, // Top Bound X
			tby = max(max(a.y, b.y), c.y) + e_cutoff,
			tbz = max(max(a.z, b.z), c.z) + e_cutoff;

		CLOCK_STOPSTART(FaceBoundTest, Face_Transform);

		rectangle rect = findRectangle(a, b, c);
		// Transform face into XY plane
		aiVector3D AB;	// Triangle edges
		aiVector3D AC;
		aiVector3D At;	// Vertices transposed onto XY plane
		aiVector3D Bt;
		aiVector3D Ct;
		aiVector3D i;	// Local reference frame vectors of the face
		aiVector3D j;
		aiVector3D k;
		aiVector3D kbar; // The k vector (z axis) transformed with the face
		float ACdoti;
		float magAB;
		float magAC;

		if (shell_type == INTRI_TF || volume) {
			// Find the local frame of the polygon ABC
			AB.x = b.x - a.x; AB.y = b.y - a.y; AB.z = b.z - a.z;
			AC.x = c.x - a.x; AC.y = c.y - a.y; AC.z = c.z - a.z;
			magAB = AB.Length();
			magAC = AC.Length();

			// i is unit vector in direction AB
			i.x = AB.x / magAB;
			i.y = AB.y / magAB;
			i.z = AB.z / magAB;
			ACdoti = AC.x*i.x + AC.y*i.y + AC.z*i.z;

			// j is in the AB / AC plane and perpendicular to i with unit length
			j.x = (AC.x - ACdoti*i.x) / magAC;
			j.y = (AC.y - ACdoti*i.y) / magAC;
			j.z = (AC.z - ACdoti*i.z) / magAC;

			// k is unit vector perpendicular
			k.x = i.y*j.z - i.z*j.y;
			k.y = i.z*j.x - i.x*j.z;
			k.z = i.x*j.y - i.y*j.x;

			// kbar is a unit vector in the z direction transformed
			kbar.x = i.z;
			kbar.y = j.z;
			kbar.z = k.z;

			// Transform polygon point into local frame
			At.x = 0; At.y = 0; At.z = 0;

			Bt.x = i.x*(b.x - a.x) + i.y*(b.y - a.y) + i.z*(b.z - a.z);
			Bt.y = j.x*(b.x - a.x) + j.y*(b.y - a.y) + j.z*(b.z - a.z);
			Bt.z = k.x*(b.x - a.x) + k.y*(b.y - a.y) + k.z*(b.z - a.z);

			Ct.x = i.x*(c.x - a.x) + i.y*(c.y - a.y) + i.z*(c.z - a.z);
			Ct.y = j.x*(c.x - a.x) + j.y*(c.y - a.y) + j.z*(c.z - a.z);
			Ct.z = k.x*(c.x - a.x) + k.y*(c.y - a.y) + k.z*(c.z - a.z);
		}

		CLOCK_STOPSTART(Face_Transform, Colour);

		// Colour calculations
		aiColor3D color = FOM;
		const unsigned int mati = m->mMaterialIndex;
		if (!image.empty() && image[mati] != nullptr) { //check for a null material
			const int
				texW = image[mati]->w,
				texH = image[mati]->h
				;

			aiVector2t<int> uvs1, uvs2, uvs3;
			uvs1.x = floor((m->mTextureCoords[0][f0].x) * texW);
			uvs1.y = floor((1 - m->mTextureCoords[0][f0].y) * texH);
			uvs2.x = floor((m->mTextureCoords[0][f1].x) * texW);
			uvs2.y = floor((1 - m->mTextureCoords[0][f1].y) * texH);
			uvs3.x = floor((m->mTextureCoords[0][f2].x) * texW);
			uvs3.y = floor((1 - m->mTextureCoords[0][f2].y) * texH);

			const Uint32
				RGBVertex1 = getpixel(image[mati], uvs1.x, uvs1.y),
				RGBVertex2 = getpixel(image[mati], uvs2.x, uvs2.y),
				RGBVertex3 = getpixel(image[mati], uvs3.x, uvs3.y)
				;

			aiColor4t<Uint8> pixRaw[3];

			const SDL_PixelFormat* format = image[mati]->format;

			SDL_GetRGBA(RGBVertex1, format, &pixRaw[0].r, &pixRaw[0].g, &pixRaw[0].b, &pixRaw[0].a);
			SDL_GetRGBA(RGBVertex2, format, &pixRaw[1].r, &pixRaw[1].g, &pixRaw[1].b, &pixRaw[1].a);
			SDL_GetRGBA(RGBVertex3, format, &pixRaw[2].r, &pixRaw[2].g, &pixRaw[2].b, &pixRaw[2].a);

			aiColor3D pix[3];

			//unrolled for -verbosiy- reasons
			pix[0].r = pixRaw[0].r / 255.0;
			pix[0].g = pixRaw[0].g / 255.0;
			pix[0].b = pixRaw[0].b / 255.0;
			pix[1].r = pixRaw[1].r / 255.0;
			pix[1].g = pixRaw[1].g / 255.0;
			pix[1].b = pixRaw[1].b / 255.0;
			pix[2].r = pixRaw[2].r / 255.0;
			pix[2].g = pixRaw[2].g / 255.0;
			pix[2].b = pixRaw[2].b / 255.0;
			pix[0] /= pixRaw[0].a / 255.0;
			pix[1] /= pixRaw[1].a / 255.0;
			pix[2] /= pixRaw[2].a / 255.0;

			/* R.I.P.
			for (int p = 0; p < 3; p++) {
			for (int c = 0; c < 3; c++) {
			pix[p][c] = pixRaw[p][c]/255.0;
			}
			pix[p] /= pixRaw[p].a/255.0;
			}
			*/

			color = (pix[0] + pix[1] + pix[2]) / 3;

		}
		else if (materialColors.size() > 0) {
			color = aiColor3D(
				materialColors[mati].r,
				materialColors[mati].g,
				materialColors[mati].b
			);

		}
		else {
			color = *lit;
		}
		float e;
		CLOCK_STOPSTART(Colour, Loop);
		// =============================================================> FOR EACH LED
		for (int ledNo = 0; ledNo < locations.size(); ledNo++) {
			CLOCK_STOPSTART(Loop, LedPre);

			e = e_cutoff + 0.5;

			CLOCK_STOPSTART(LedPre, FaceBoundTest);

			if (
				locations[ledNo].x < bbx || locations[ledNo].x > tbx ||
				locations[ledNo].y < bby || locations[ledNo].y > tby ||
				locations[ledNo].z < bbz || locations[ledNo].z > tbz
				) {
#if PROFILE
				//faceBoundSkip++;
#endif
				CLOCK_STOPSTART(FaceBoundTest, Loop);
				continue;//to next led in for loop
			}
			CLOCK_STOPSTART(FaceBoundTest, Volume);

			// Volume test
			if (volume) {
				int intersection = volume_test(a, At, Bt, Ct, locations[ledNo], i, j, k, kbar, intri_cutoff_volume);
				if (intersection == POSITIVE_INTERSECTION) processing_progress::above(ledNo);
				else if (intersection == NEGATIVE_INTERSECTION) processing_progress::below(ledNo);
			}

			CLOCK_STOPSTART(Volume, Intri)

				// Intri test
				if (shell_type == INTRI_TF) e = intri_tf(a, At, Bt, Ct, locations[ledNo], i, j, k, intri_cutoff);
				else if (shell_type == INTRI) e = intri(&a, &b, &c, &locations[ledNo]);
				else if (shell_type == NOT_INTRI) e = not_intri(&a, &b, &c, &locations[ledNo]);
				if (e < 0.0 || e > e_cutoff) {
#if PROFILE
					//e_cutoffSkip++;
#endif
					CLOCK_STOPSTART(Intri, Loop);
					continue;
				}
				e = e / minMax * 10;

				CLOCK_STOPSTART(Intri, Intensity);

				float intensity = pow((float)::decay, -(float)e);

				CLOCK_STOPSTART(Intensity, Colour);

				processing_progress::add_led(ledNo, color);
#if PROFILE
				//noSkip++;
#endif

				CLOCK_STOPSTART(Colour, Loop);
		}// for each led
		CLOCK_STOPSTART(Loop, Display);

		//if (progress) quickProgress(do_face.face_index, m->mNumFaces, do_face.mesh_index);

		CLOCK_STOPSTART(Display, Loop);
	}
}

vector<aiColor3D> sceneToRGB(aiScene* scene, vector<aiVector3D>& ledxyz, string filedir) {
	CLOCK_START(Total);
	CLOCK_START(Setup);

	//Statistics
	if (!silent) cout << "Number of meshes found in file: " << scene->mNumMeshes << endl;
	if (scene->mNumMeshes == 0) {
		if (!silent) cout << "No meshes in file" << endl;
		exit(EXIT_FAILURE);
	}
	//Count total faces and verticies...why?
	int noOfFaces = 0;
	int noOfVerticies = 0;
	for (unsigned int i = 0; i < scene->mNumMeshes; i++) {
		noOfFaces += scene->mMeshes[i]->mNumFaces;
		noOfVerticies += scene->mMeshes[i]->mNumVertices;
	}
	if (!silent) cout << "Number of faces: " << noOfFaces << endl;
	if (!silent) cout << "Number of vertices: " << noOfVerticies << endl;

	//Scale scene to the size of the LED Cube
	scaleToLEDCubeSize(scene, ledxyz);

	//find minMax (minimum maximum value)
	float maxX = 0; float maxY = 0; float maxZ = 0;
	for (int n = 0; n < scene->mNumMeshes; n++) {
		const aiMesh* m = scene->mMeshes[n];
		for (int i = 0; i < m->mNumVertices; i++) {
			const aiVector3D& v = m->mVertices[i];
			maxX = max(maxX, v.x);
			maxY = max(maxY, v.y);
			maxZ = max(maxZ, v.z);
		}
	}
	float minMax = min(min(maxX, maxY), maxZ);
	if (!silent) cout << "Maximum {x,y,z} = " << maxX << "," << maxY << "," << maxZ << endl;

	vector<aiString> texturePath(scene->mNumMaterials);

	//If Colors are embedded in the 3D graphics file the color is stored in materialColors
	//when there is an external texture file the texturefile path is saved.
	//needs to be initialized to scene->mNumMaterials size so when texturePath[i] is null materialColors[i] will have a color
	vector<aiColor3D> materialColors(scene->mNumMaterials, FOM);
	if (scene->HasMaterials()) {
		for (unsigned int i = 0; i < scene->mNumMaterials; i++) {
			const aiMaterial* material = scene->mMaterials[i];

			if (material->GetTextureCount(aiTextureType_DIFFUSE) > 0 && material->GetTexture(aiTextureType_DIFFUSE, 0, &texturePath[i]) == AI_SUCCESS) {
				if (!silent && progress) cout << "Texture file found: " << texturePath[i].C_Str() << endl;
			} else {
				if (!silent && progress) cout << "There is a material with no texture file, ";
				aiColor4D d;
				if (AI_SUCCESS == aiGetMaterialColor(scene->mMaterials[i], AI_MATKEY_COLOR_DIFFUSE, &d)) {
					materialColors[i] = aiColor3D(d.r, d.g, d.b)*d.a;
					if (!silent && progress) cout << "but there is a solid material color." << endl;
				} else if (!silent && progress) cout << "and there is no solid material color." << endl;
			}
		}
	}

	//Load the texture file(s)
	vector<int> texture_width;
	vector<int> texture_height;
	vector<SDL_Surface*> image(texturePath.size(), nullptr);

	for (int textureNum = 0; textureNum < texturePath.size(); textureNum++) { //for multiple texture files
		if (texturePath[textureNum] != (aiString)"") {
			string loadLocation = filedir + "\\" + string(texturePath[textureNum].C_Str()).c_str();
			if (!silent) cout << "loading: " << loadLocation << endl;
			string extension = loadLocation.substr(loadLocation.find_last_of(".") + 1);
			if (extension == "jpg" || extension == "JPG") {
				IMG_Init(IMG_INIT_JPG);
			}
			else if (extension == "png" || extension == "PNG") {
				IMG_Init(IMG_INIT_PNG);
			}
			else if (extension == "tif" || extension == "tiff" || extension == "TIFF") {
				IMG_Init(IMG_INIT_TIF);
			}
			else {
				if (!silent) cout << "Unsupported texture file format: " << loadLocation.substr(loadLocation.find_last_of(".") + 1) << endl << "Needs to be .jpg or .png" << endl;
				exit(EXIT_FAILURE);
			}
			/*if (loadLocation.substr(loadLocation.find_last_of(".") + 1) == "jpg") {
				IMG_Init(IMG_INIT_JPG);
			} else if (loadLocation.substr(loadLocation.find_last_of(".") + 1) == "png") {
				IMG_Init(IMG_INIT_PNG);
			} else {
				if (!silent) cout << "Unsupported texture file format: " << loadLocation.substr(loadLocation.find_last_of(".") + 1) << endl << "Needs to be .jpg or .png" << endl;
				exit(EXIT_FAILURE);
			}*/

			image[textureNum] = IMG_Load(loadLocation.c_str());

			if (image[textureNum] == nullptr) {
				if (!silent) cout << "IMG_Load: " << IMG_GetError() << "\n";
				exit(EXIT_FAILURE);
			}
		}
	}

	vector<int> below(ledxyz.size(), 0);
	vector<int> above(ledxyz.size(), 0);
	/*
	below.reserve(ledxyz.size());
	above.reserve(ledxyz.size());
	for (int i = 0; i < ledxyz.size(); i++) {
		below.push_back(0);
		above.push_back(0);
	}
	*/

	vector<aiColor3D> ledrgb(ledxyz.size(), aiColor3D());// init to black
	processing_progress::init(scene, ledrgb, above, below);

	vector<thread> threads;
	for (int i = 0; i < nthreads; i++) {
		threads.emplace_back(colour_face, ledxyz, scene, minMax, materialColors, texturePath, image);
		cout << "Thread " << i << " launched" << endl;
	}
	for (int i = 0; i < nthreads; i++) threads[i].join();
	ledrgb = processing_progress::colours;

#if PROFILE
	unsigned long int
		faceBoundSkip = 0,
		e_cutoffSkip = 0,
		noSkip = 0;
#endif

	CLOCK_STOPSTART(Setup, Loop);
	// ============================================================> FOR EACH MESH
	
		//disp times
	if (!silent) {
#if PROFILE
		auto sum = faceBoundSkip + e_cutoff + noSkip;
		cout << endl;
		cout << "face-bound test skipped " << setw(10) << faceBoundSkip << " executions (" << 100.0*faceBoundSkip / sum << "%)" << endl;
		cout << "e_cutoff test skipped   " << setw(10) << e_cutoffSkip << " executions (" << 100.0*e_cutoffSkip / sum << "%)" << endl;
		cout << "full color set          " << setw(10) << noSkip << " executions (" << 100.0*noSkip / sum << "%)" << endl;
		cout << "faces X verticies:      " << setw(10) << sum << endl;
#endif

		int nLit = 0;
		for (auto c : ledrgb) {
			if (c.r && c.g && c.b) nLit++;
		}
		cout << "number lit: " << nLit << endl;

		CLOCK_DISP(Setup);
		CLOCK_DISP(MeshPre);
		CLOCK_DISP(FacePre);
		CLOCK_DISP(Face_Transform);
		CLOCK_DISP(FaceBoundTest);
		CLOCK_DISP(LedPre);
		CLOCK_DISP(Volume);
		CLOCK_DISP(Intri);
		CLOCK_DISP(Intensity);
		CLOCK_DISP(Colour);
		CLOCK_DISP_RUNNING(Display);
		CLOCK_DISP(Loop);
		CLOCK_DISP_RUNNING(Total);

		CLOCK_STOPSTART(Display, Loop);

	}
	CLOCK_STOPSTART(Loop, IntensityAndVolume);
	 // Find maximum intensity for scaling
	float max_intensity = 0.0;
	for (aiColor3D led_colour : ledrgb) {
		float led_max = max(max(led_colour.r, led_colour.b), led_colour.g);
		if (led_max > max_intensity) max_intensity = led_max;
	}
	if (!silent) cout << endl << "Highest intensity: " << max_intensity << endl;
	// Normalise intensity and perform volume lighting
	for (int i = 0; i < ledrgb.size(); i++) {
		ledrgb[i] /= max_intensity;
		if (above[i] % 2 > 0 && below[i] % 2 > 0) {
			ledrgb[i].r = (ledrgb[i].r + vlit->r) / 2;
			ledrgb[i].g = (ledrgb[i].g + vlit->g) / 2;
			ledrgb[i].b = (ledrgb[i].b + vlit->b) / 2;
		}
		// Sometimes LEDs can be blinding, so you can scale the brightness back
		ledrgb[i].r *= max_brightness;
		ledrgb[i].g *= max_brightness;
		ledrgb[i].b *= max_brightness;
	}
	CLOCK_STOP(IntensityAndVolume);
	CLOCK_STOP(Total);

	if (!silent) cout << endl << "Finished" << endl;
	return ledrgb;
}

int main(int argc, char* argv[]) {
	if (argc == 3 && argv[1][0] == 'p') {
		string proportion_path = argv[2];
		find_proportions(proportion_path);
	} else if (argc < 4) {
		cout << "Not enough Arguments" << std::endl;
		return -1;
	} else {
		int param = 4;
		while (param < argc) {
			switch (argv[param][0]) {
			case 'l':			// Loud mode
				silent = false;
				param++;
				break;
			case 'v':			// volume filling
				volume = true;
				param++;
				break;
			case 's':			// shell lighting type
				shell_type = atoi(argv[param + 1]);
				param += 2;
				break;
			case 'd':
				::decay = strtof(argv[param + 1], NULL);
				param += 2;
				break;
			case 'm':
				max_brightness = strtod(argv[param + 1], NULL);
				param++;
				break;
			case 't':			// intri thresholds
				if (argv[param][1] == 'h') {
					nthreads = strtof(argv[param + 1], NULL);
				}
				else if (argv[param][1] == 'i') {
					intri_cutoff = strtof(argv[param + 1], NULL); // For shell lighting
				}
				else if (argv[param][1] == 'v') {
					intri_cutoff_volume = strtof(argv[param + 1], NULL); // For volume filling
				}
				param += 2;
				break;
			case 'c':			// Set lighting colours
				if (argv[param][1] == 's') {
					// For shell lighting, if no textures or materials are found
					lit->r = strtof(argv[param + 1], NULL);
					lit->g = strtof(argv[param + 2], NULL);
					lit->b = strtof(argv[param + 3], NULL);
					param += 4;
				}
				else if (argv[param][1] == 'v') {
					// For volume filling
					vlit->r = strtof(argv[param + 1], NULL);
					vlit->g = strtof(argv[param + 2], NULL);
					vlit->b = strtof(argv[param + 3], NULL);
					param += 4;
				}
				break;
			case 'n':			// No progress bar, for test files
				progress = false;
				param++;
				break;
			default:
				param++;
			}
		}

		clock_t start_time = clock();

		string xyz_path = argv[1]; // C:/Example/64Cube
		string model_path = argv[2]; // C:/Exacmple/simpleMan.obj
		string outputRGBLocation = argv[3]; // C:/Example/output_man

		string wkdir = model_path.substr(0, model_path.find_last_of("/\\"));
		aiScene* scene = readSceneFile(model_path);
		vector<aiVector3D> ledxyz = readXYZCSVFile(xyz_path, progress);
		vector<aiColor3D> ledrgb = sceneToRGB(scene, ledxyz, wkdir);
		writeRGBCSVFile(outputRGBLocation, ledrgb, silent);

		if (!silent) cout << "Total time: " <<setw(7)<<fixed<<setprecision(3)<< double(clock() - start_time) / CLOCKS_PER_SEC << " s." << endl;
	}

	return 0;
	
}
