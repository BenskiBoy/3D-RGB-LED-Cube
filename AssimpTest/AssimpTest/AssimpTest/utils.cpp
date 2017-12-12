#include "ben.h"

using namespace std;

//Load a 3D graphics file
aiScene* readSceneFile(string file) {
	////////////////////////////////////
	// default pp steps
	unsigned int ppsteps = (0
		| aiProcess_CalcTangentSpace // calculate tangents and bitangents if possible
									 //| aiProcess_JoinIdenticalVertices // join identical vertices/ optimize indexing
		| aiProcess_ValidateDataStructure // perform a full validation of the loader's output
		| aiProcess_ImproveCacheLocality // improve the cache locality of the output vertices
		| aiProcess_RemoveRedundantMaterials // remove redundant materials
											 //| aiProcess_FindDegenerates // remove degenerated polygons from the import
		| aiProcess_FindInvalidData // detect invalid model data, such as invalid normal vectors
		| aiProcess_GenUVCoords // convert spherical, cylindrical, box and planar mapping to proper UVs
		| aiProcess_TransformUVCoords // preprocess UV transformations (scaling, translation ...)
		| aiProcess_FindInstances // search for instanced meshes and remove them by references to one master
		| aiProcess_LimitBoneWeights // limit bone weights to 4 per vertex
									 //| aiProcess_OptimizeMeshes // join small meshes, if possible;
		| aiProcess_SplitByBoneCount // split meshes with too many bones. Necessary for our (limited) hardware skinning shader

		| aiProcess_GenSmoothNormals // generate smooth normal vectors if not existing
		| aiProcess_SplitLargeMeshes // split large, unrenderable meshes into submeshes
		| aiProcess_Triangulate // triangulate polygons with more than 3 edges
								//| aiProcess_ConvertToLeftHanded // convert everything to D3D left handed space
		| aiProcess_SortByPType // make 'clean' meshes which consist of a single typ of primitives
		);

	aiPropertyStore* props = aiCreatePropertyStore();

	aiSetImportPropertyInteger(props, AI_CONFIG_IMPORT_TER_MAKE_UVS, 1);
	aiSetImportPropertyFloat(props, AI_CONFIG_PP_GSN_MAX_SMOOTHING_ANGLE, 80.f);
	aiSetImportPropertyInteger(props, AI_CONFIG_PP_SBP_REMOVE, aiPrimitiveType_LINE | aiPrimitiveType_POINT);
	aiSetImportPropertyInteger(props, AI_CONFIG_GLOB_MEASURE_TIME, 1);
	aiSetImportPropertyInteger(props, AI_CONFIG_IMPORT_AC_EVAL_SUBDIVISION, true);
	aiSetImportPropertyInteger(props, AI_CONFIG_PP_SLM_TRIANGLE_LIMIT, 10000000000);
	//aiSetImportPropertyInteger(props, AI_CONFIG_PP_PTV_KEEP_HIERARCHY, 1);

	// Call ASSIMPs C-API to load the file
	auto scene = (aiScene*)aiImportFileExWithProperties(
		file.c_str(),
		ppsteps,
		NULL,
		props
	);

	//aiReleasePropertyStore(props);//TODO should we clean up after?

	if (!scene) {
		//cout << aiGetErrorString() << endl;
		exit(EXIT_FAILURE);
	}
	else return scene;
}

//possibly reduce a copy op
aiColor3D& operator+=(aiColor3D& lhs, const aiColor3D& rhs) {
	lhs.r += rhs.r;
	lhs.g += rhs.g;
	lhs.b += rhs.b;
	return lhs;
}

aiColor3D& operator/=(aiColor3D& lhs, const float rhs) {
	lhs.r /= rhs;
	lhs.g /= rhs;
	lhs.b /= rhs;
	return lhs;
}

aiColor3D& operator/( const aiColor3D lhs, const float rhs) {
	return aiColor3D(lhs.r / rhs, lhs.g / rhs, lhs.b / rhs);
}

vector<aiVector3D> readXYZCSVFile(string name, bool p) {
	vector<aiVector3D> ledxyz;
	ifstream infile(name);
	while (true) {
		double x, y, z;
		infile >> x; infile.ignore(256, ',');
		infile >> y; infile.ignore(256, ',');
		infile >> z; infile.ignore(256, '\n');
		if (infile) ledxyz.emplace_back(x, y, z);
		else break;
		if (p && ledxyz.size() % 1000 == 0) cout << ledxyz.size() << '\r';
	}
	cout << ".xyz file successfully read " << ledxyz.size() << " positions." << endl;
	return ledxyz;
}


//Takes the scene that is loaded and scales the first mesh to the most limiting dimension.
void scaleToLEDCubeSize(const aiScene* scene, vector<aiVector3D>& ledxyz) {

	//Initialize vectors to size of number of meshes
	vector<float> maxXv(scene->mNumMeshes, -(numeric_limits<float>::max()));
	vector<float> maxYv(scene->mNumMeshes, -(numeric_limits<float>::max()));
	vector<float> maxZv(scene->mNumMeshes, -(numeric_limits<float>::max()));

	vector<float> minXv(scene->mNumMeshes, (numeric_limits<float>::max()));
	vector<float> minYv(scene->mNumMeshes, (numeric_limits<float>::max()));
	vector<float> minZv(scene->mNumMeshes, (numeric_limits<float>::max()));

	//Determine highest and lowest verticy points
	for (int n = 0; n < scene->mNumMeshes; n++) {
		for (unsigned int i = 0; i < scene->mMeshes[n]->mNumVertices; i++) {
			if (scene->mMeshes[n]->mVertices[i].x > maxXv[n])
				maxXv[n] = scene->mMeshes[n]->mVertices[i].x;
			if (scene->mMeshes[n]->mVertices[i].y > maxYv[n])
				maxYv[n] = scene->mMeshes[n]->mVertices[i].y;
			if (scene->mMeshes[n]->mVertices[i].z > maxZv[n])
				maxZv[n] = scene->mMeshes[n]->mVertices[i].z;

			if (scene->mMeshes[n]->mVertices[i].x < minXv[n])
				minXv[n] = scene->mMeshes[n]->mVertices[i].x;
			if (scene->mMeshes[n]->mVertices[i].y < minYv[n])
				minYv[n] = scene->mMeshes[n]->mVertices[i].y;
			if (scene->mMeshes[n]->mVertices[i].z < minZv[n])
				minZv[n] = scene->mMeshes[n]->mVertices[i].z;
		}
	}

	double minXPos = *min_element(minXv.begin(), minXv.end());
	double minYPos = *min_element(minYv.begin(), minYv.end());
	double minZPos = *min_element(minZv.begin(), minZv.end());

	for (int n = 0; n < scene->mNumMeshes; n++) {
		for (unsigned int i = 0; i < scene->mMeshes[n]->mNumVertices; i++) {
			//Lowest position is located at origin
			scene->mMeshes[n]->mVertices[i].x -= minXPos;
			scene->mMeshes[n]->mVertices[i].y -= minYPos;
			scene->mMeshes[n]->mVertices[i].z -= minZPos;
		}
	}

	//Initialize vectors to size of number of meshes
	fill(maxXv.begin(), maxXv.end(), -(numeric_limits<float>::max()));
	fill(maxYv.begin(), maxYv.end(), -(numeric_limits<float>::max()));
	fill(maxZv.begin(), maxZv.end(), -(numeric_limits<float>::max()));

	fill(minXv.begin(), minXv.end(), (numeric_limits<float>::max()));
	fill(minYv.begin(), minYv.end(), (numeric_limits<float>::max()));
	fill(minZv.begin(), minZv.end(), (numeric_limits<float>::max()));

	//Determine highest and lowest verticy points
	for (int n = 0; n < scene->mNumMeshes; n++) {
		for (unsigned int i = 0; i < scene->mMeshes[n]->mNumVertices; i++) {
			if (scene->mMeshes[n]->mVertices[i].x > maxXv[n])
				maxXv[n] = scene->mMeshes[n]->mVertices[i].x;
			if (scene->mMeshes[n]->mVertices[i].y > maxYv[n])
				maxYv[n] = scene->mMeshes[n]->mVertices[i].y;
			if (scene->mMeshes[n]->mVertices[i].z > maxZv[n])
				maxZv[n] = scene->mMeshes[n]->mVertices[i].z;

			if (scene->mMeshes[n]->mVertices[i].x < minXv[n])
				minXv[n] = scene->mMeshes[n]->mVertices[i].x;
			if (scene->mMeshes[n]->mVertices[i].y < minYv[n])
				minYv[n] = scene->mMeshes[n]->mVertices[i].y;
			if (scene->mMeshes[n]->mVertices[i].z < minZv[n])
				minZv[n] = scene->mMeshes[n]->mVertices[i].z;
		}
	}

	/*for (int n = 0; n < scene->mNumMeshes; n++) {
	cout << "Mesh [" << n << "]: " << "New Max X: " << maxXv[n] << "New Max Y: " << maxYv[n] << "New Max Z: " << maxZv[n] << endl;
	cout << "Mesh [" << n << "]: " << "New Min X: " << minXv[n] << "New Min Y: " << minYv[n] << "New Min Z: " << minZv[n] << endl;
	}*/

	float maxX = -(numeric_limits<float>::max());
	float maxY = -(numeric_limits<float>::max());
	float maxZ = -(numeric_limits<float>::max());
	float minX = numeric_limits<float>::max();
	float minY = numeric_limits<float>::max();
	float minZ = numeric_limits<float>::max();

	//Determine adjusted highest and lowest verticy points
	for (int n = 0; n < scene->mNumMeshes; n++) {
		for (unsigned int i = 0; i < scene->mMeshes[n]->mNumVertices; i++) {
			if (scene->mMeshes[n]->mVertices[i].x > maxXv[n])
				maxXv[n] = scene->mMeshes[n]->mVertices[i].x;
			if (scene->mMeshes[n]->mVertices[i].y > maxYv[n])
				maxYv[n] = scene->mMeshes[n]->mVertices[i].y;
			if (scene->mMeshes[n]->mVertices[i].z > maxZv[n])
				maxZv[n] = scene->mMeshes[n]->mVertices[i].z;

			if (scene->mMeshes[n]->mVertices[i].x < minXv[n])
				minXv[n] = scene->mMeshes[n]->mVertices[i].x;
			if (scene->mMeshes[n]->mVertices[i].y < minYv[n])
				minYv[n] = scene->mMeshes[n]->mVertices[i].y;
			if (scene->mMeshes[n]->mVertices[i].z < minZv[n])
				minZv[n] = scene->mMeshes[n]->mVertices[i].z;
		}
	}


	/*for (int n = 0; n < scene->mNumMeshes; n++) {
	cout << "Mesh [" << n << "]: " << "New Max X: " << maxXv[n] << "New Max Y: " << maxYv[n] << "New Max Z: " << maxZv[n] << endl;
	cout << "Mesh [" << n << "]: " << "New Min X: " << minXv[n] << "New Min Y: " << minYv[n] << "New Min Z: " << minZv[n] << endl;
	}*/

	//Get the largest x, y and z LED positions
	float maxLEDPositionX = 0;
	float maxLEDPositionY = 0;
	float maxLEDPositionZ = 0;

	for (int i = 0; i < ledxyz.size(); i++) {
		if (ledxyz.at(i).x > maxLEDPositionX) {
			maxLEDPositionX = ledxyz.at(i).x;
		}
		if (ledxyz.at(i).y > maxLEDPositionY) {
			maxLEDPositionY = ledxyz.at(i).y;
		}
		if (ledxyz.at(i).z > maxLEDPositionZ) {
			maxLEDPositionZ = ledxyz.at(i).z;
		}
	}

	vector<float> scalingFactor(scene->mNumMeshes, 0);
	for (int n = 0; n < scene->mNumMeshes; n++) {
		float xScalingFactor = maxLEDPositionX / maxXv[n];
		float yScalingFactor = maxLEDPositionY / maxYv[n];
		float zScalingFactor = maxLEDPositionZ / maxZv[n];

		scalingFactor[n] = min({ xScalingFactor, yScalingFactor, zScalingFactor });
	}

	double finalScalingFactor = *min_element(scalingFactor.begin(), scalingFactor.end());

	for (int n = 0; n < scene->mNumMeshes; n++) {
		for (unsigned int i = 0; i < scene->mMeshes[n]->mNumVertices; i++) {
			//Lowest position is located at origin
			scene->mMeshes[n]->mVertices[i].x *= finalScalingFactor;
			scene->mMeshes[n]->mVertices[i].y *= finalScalingFactor;
			scene->mMeshes[n]->mVertices[i].z *= finalScalingFactor;
		}
	}
}

//writes the scenes verticie locations to csv file
//NOTE: Only a linefeed is written, no carriage return. May be issue for interpreters of generatred file
void writeRGBCSVFile(string file, vector<aiColor3D>& ledrgb, bool silent) {
	if (!silent) cout << "Writing To File: " << file << endl << endl;
	ofstream rgbfile;
	rgbfile.open(file, ios::binary);

	for (unsigned int i = 0; i < ledrgb.size(); i++) {
		rgbfile << setprecision(6) << fixed;
		rgbfile << ledrgb[i].r << ",";
		rgbfile << ledrgb[i].g << ",";
		rgbfile << ledrgb[i].b << "\r\n";//endl;

										 //std::cout << ledrgb[i].r << " " << ledrgb[i].g << " " << ledrgb[i].b << std::endl;
	}
	if(!silent) cout << endl << "Finished Writing to File" << endl;
}

Uint32 getpixel(SDL_Surface *surface, int x, int y) {
	int bpp = surface->format->BytesPerPixel;
	/* Here p is the address to the pixel we want to retrieve */
	Uint8 *p = (Uint8 *)surface->pixels + y * surface->pitch + x * bpp;
	switch (bpp) {
	case 1:
		return *p;
		break;
	case 2:
		return *(Uint16 *)p;
		break;
	case 3:
		if (SDL_BYTEORDER == SDL_BIG_ENDIAN)
			return p[0] << 16 | p[1] << 8 | p[2];
		else
			return p[0] | p[1] << 8 | p[2] << 16;
		break;
	case 4:
		return *(Uint32 *)p;
		break;
	default:
		return 0;// shouldn't happen, but avoids warnings
	}
}

string find_proportions(string path) {
	aiScene * model = readSceneFile(path);
	string csv = "";

	float maxX = -(numeric_limits<float>::max());
	float maxY = -(numeric_limits<float>::max());
	float maxZ = -(numeric_limits<float>::max());
	float minX = numeric_limits<float>::max();
	float minY = numeric_limits<float>::max();
	float minZ = numeric_limits<float>::max();

	//Determine adjusted highest and lowest verticy points
	for (int n = 0; n < model->mNumMeshes; n++) {
		for (unsigned int i = 0; i < model->mMeshes[n]->mNumVertices; i++) {
			// Maxs
			if (model->mMeshes[n]->mVertices[i].x > maxX)
				maxX = model->mMeshes[n]->mVertices[i].x;
			if (model->mMeshes[n]->mVertices[i].y > maxY)
				maxY = model->mMeshes[n]->mVertices[i].y;
			if (model->mMeshes[n]->mVertices[i].z > maxZ)
				maxZ = model->mMeshes[n]->mVertices[i].z;
			// Mins
			if (model->mMeshes[n]->mVertices[i].x < minX)
				minX = model->mMeshes[n]->mVertices[i].x;
			if (model->mMeshes[n]->mVertices[i].y < minY)
				minY = model->mMeshes[n]->mVertices[i].y;
			if (model->mMeshes[n]->mVertices[i].z < minZ)
				minZ = model->mMeshes[n]->mVertices[i].z;
		}
	}
	float xdif = maxX - minX;
	float ydif = maxY - minY;
	float zdif = maxZ - minZ;
	float mindim = min(min(xdif, ydif), zdif);
	xdif /= mindim;
	ydif /= mindim;
	zdif /= mindim;
	cout << "Proportions x : y : z = " << xdif << " : " << ydif << " : " << zdif << endl;
	csv = to_string(xdif) + "," + to_string(ydif) + "," + to_string(zdif);
	return csv;
}

std::mutex			processing_progress::progress_mu;
progress_index		processing_progress::progress;
aiScene *			processing_progress::scene;
vector<aiColor3D>	processing_progress::colours;
vector<mutex> *		processing_progress::led_mus;
vector<int>			processing_progress::n_above;
vector<int>			processing_progress::n_below;

void processing_progress::init(aiScene * inScene, std::vector<aiColor3D>  inColours, vector<int> & inAbove, vector<int> & inBelow) {
	processing_progress::scene = inScene;
	processing_progress::colours = inColours;
	led_mus = new vector<mutex>(colours.size());
	progress.mesh_index = 0;
	progress.face_index = 0;
	n_above = inAbove;
	n_below = inBelow;
}

progress_index processing_progress::next_face() {
	// Returns indexes then increments
	progress_mu.lock();
	if (progress.mesh_index == scene->mNumMeshes) {
		progress_mu.unlock();
		return { -1,-1 }; // Task finished
	}
	progress_index current = progress; // Save then increment
	progress.face_index++;
	if (progress.face_index == scene->mMeshes[progress.mesh_index]->mNumFaces) {
		progress.mesh_index++;
		progress.face_index = 0;
		cout << "Mesh no. " << progress.mesh_index - 1 << " done" << endl;
	}
	// Unlock and return indexes
	progress_mu.unlock();
	return current;
}

void processing_progress::add_led(int index, aiColor3D addColour) {
	(*led_mus)[index].lock();
	colours[index] += addColour;
	(*led_mus)[index].unlock();
}

void processing_progress::above(int index) {
	(*led_mus)[index].lock();
	n_above[index]++;
	(*led_mus)[index].unlock();
}

void processing_progress::below(int index) {
	(*led_mus)[index].lock();
	n_below[index]++;
	(*led_mus)[index].unlock();
}