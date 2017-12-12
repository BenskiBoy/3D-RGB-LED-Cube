#ifndef BENNIBOI
#define BENNIBOI

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

aiScene* readSceneFile(std::string file);
aiColor3D& operator+=(aiColor3D& lhs, const aiColor3D& rhs);
aiColor3D& operator/=(aiColor3D& lhs, const float rhs);
aiColor3D& operator/(const aiColor3D lhs, const float rhs);
std::vector<aiVector3D> readXYZCSVFile(std::string name, bool p);
//i thought the progress bar may have been slowing it down, but probably not
template<typename T>
void quickProgress(int p, int t, T m) {
	static const int barWidth = 70;
	if (p%max(t / barWidth, 1) == 0) {
		cout << m << "[";
		unsigned int pos = barWidth * p / t;
		for (unsigned int i = 0; i < barWidth; ++i) {
			if (i < pos) cout << "=";
			else if (i == pos) cout << ">";
		}
		cout << "] " << setw(3) << int(100 * p / t) << " %\r";
		cout.flush();
	}
	if (p == t) cout << endl;
}
void scaleToLEDCubeSize(const aiScene* scene, std::vector<aiVector3D>& ledxyz);
void writeRGBCSVFile(std::string file, std::vector<aiColor3D>& ledrgb, bool silent);
Uint32 getpixel(SDL_Surface *surface, int x, int y);
std::string find_proportions(std::string path);

struct progress_index {
	int mesh_index;
	int face_index;
};

// Multi-threading
namespace processing_progress {
	extern std::mutex progress_mu;
	extern progress_index progress;
	extern aiScene * scene;
	extern std::vector<aiColor3D> colours;
	extern std::vector<std::mutex> * led_mus;
	extern std::vector<int> n_above;
	extern std::vector<int> n_below;
	void init(aiScene * inScene, std::vector<aiColor3D> inColours, std::vector<int> & inAbove, std::vector<int> & inBelow);
	progress_index next_face();
	void add_led(int index, aiColor3D addColour);
	void above(int index);
	void below(int index);
};

#endif