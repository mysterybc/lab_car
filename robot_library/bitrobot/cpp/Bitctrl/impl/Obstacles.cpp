#include "Obstacles.hpp"
#include "ssconfig.hpp"
#include <fstream>

namespace impl {


size_t MapData::load(const std::string& name, bool append) {
	std::ifstream file(name.c_str());
	sscfg::ConfigFile conf = sscfg::ConfigFile::load(file);

	if (!append) {
		clear();
	}

	size_t n_old = size();
	if (conf.n_items() > 0) {
		std::vector<std::string> prefix;
		conf.get("obList", prefix);
		float unit = 1.0f;
		if (conf.exist("obUnit")) {
			conf.get("obUnit", unit);
		}
		obUnit = unit;
		
		size_t n = prefix.size();
		for (size_t i = 0; i < n; ++i) {
			// Load Circle
			if (conf.exist(prefix[i] + "_CircleX")) {
				std::vector<float> x, y, r;
				conf.get(prefix[i] + "_CircleX", x);
				conf.get(prefix[i] + "_CircleY", y);
				conf.get(prefix[i] + "_CircleR", r);
				if (x.size() != y.size() || x.size() != r.size()) {
					printf("Warning: Get mismatched Circle Obstacle for %s\n", prefix[i].c_str());
				}
				else {
                    Circle one;
					for (size_t k = 0; k < x.size(); ++k) {
                        one.q = { x[k] / unit, y[k] / unit };
						one.r = r[k] / unit;
						obPoint.push_back(one);
					}
				}
			}

            // Load Lines
            if (conf.exist(prefix[i] + "_LineX")) {
                std::vector<float> x, y;
                std::vector<int> num;
                conf.get(prefix[i] + "_LineX", x);
                conf.get(prefix[i] + "_LineY", y);
                conf.get(prefix[i] + "_LineNum", num);

                if (x.size() != y.size() || num.size() == 0) {
                    printf("Warning: Get mismatched Line Obstacle for %s\n",  prefix[i].c_str());
                }
                else {
                    size_t nLine = num.size();
                    size_t base = 0;
                    for (size_t i = 0; i < nLine; ++i){
                        size_t nSeg = num[i];
                        for (size_t j = 0; j < nSeg - 1; ++j){
                            Point p = { x[base + j] / unit, y[base + j] / unit };
                            Point q = { x[base + j + 1] / unit, y[base + j + 1] / unit };
                            obLine.push_back(Line{p, q});
                        }
                        base += nSeg;
                    }
                }
            }

			// Load Poly
			if (conf.exist(prefix[i] + "_CPolyX")) {
				std::vector<float> px, py;
				std::vector<int>   num;
				conf.get(prefix[i] + "_CPolyX", px);
				conf.get(prefix[i] + "_CPolyY", py);
				conf.get(prefix[i] + "_CPolyNum", num);
				if (px.size() != py.size() || px.size() < num.size()) {
					printf("Warning: Get mismatching CPoly Obstacle for %s\n", prefix[i].c_str());
				}
				else {
					size_t npoly = num.size();
					size_t base = 0;
					for (size_t i = 0; i < npoly; ++i) {
						CPoly one(num[i]);
						for (int j = 0; j < num[i]; ++j) {
                            one[j] = Point{ px[base + j] / unit, py[base + j] / unit };
						}
						
						// Check if it is closed
						// if not, close it
						Point& p0 = *one.begin();
						Point& p1 = *(one.end() - 1);
						if (p0.x != p1.x || p0.y != p1.y) {
							one.push_back(p0);
						}
						obCPoly.push_back(one);
						base += num[i];
					}
				}
			}
		}
	}
	return size() - n_old;
}


struct BMPGen {
    enum { FILE_HEADER_SIZE = 14, INFO_HEADER_SIZE = 40, BYTES_PER_PIXEL = 3};
    void generateBitmapImage(unsigned char* image, int height, int width, char* imageFileName);

    unsigned char* createBitmapFileHeader(int height, int stride);
    unsigned char* createBitmapInfoHeader(int height, int width);
};


void ObGrid::save_grid_image(const std::string& fname) {
    int nrow = coord.nrow;
    int ncol = coord.ncol;
    std::vector<unsigned char> image(nrow * ncol * 3, 0);

    // BMP image format is written from bottom to top...
    for (int r = 0; r < nrow; ++r) {
        for (int c = 0; c < ncol; ++c) {
            int index = coord(r, c);
            int indexImage = r * ncol * 3 + c * 3;
            if (indexImage < nrow * ncol * 3) {
                image[indexImage + 2] = obgrid[index] == 0 ? 0 : 255; // red
                image[indexImage + 1] = obgrid[index] == 0 ? 0 : 255; // green
                image[indexImage + 0] = obgrid[index] == 0 ? 0 : 255; // blue
            }
            else {
                puts("Error generating image");
            }
        }
    }

    BMPGen gen;
    gen.generateBitmapImage((unsigned char*)&image[0], nrow, ncol, (char*)fname.c_str());
}

void ObGrid::save_grid_path(const std::vector<int>& path, const std::string& fname) {
    int nrow = coord.nrow;
    int ncol = coord.ncol;
    unsigned char* image = new unsigned char[nrow * ncol * 3];

    // BMP image format is written from bottom to top...
    for (int r = 0; r < nrow; ++r) {
        for (int c = 0; c < ncol; ++c) {
            int index = coord(r, c);
            int indexImage = r * ncol * 3 + c * 3;
            if (indexImage < nrow * ncol * 3) {
                image[indexImage + 2] = obgrid[index] == 0 ? 0 : 255; // red
                image[indexImage + 1] = obgrid[index] == 0 ? 0 : 255; // green
                image[indexImage + 0] = obgrid[index] == 0 ? 0 : 255; // blue
            }
            else {
                puts("Error generating image");
            }
        }
    }
    for (unsigned k = 0; k < path.size(); ++k) {
        int index = path[k];
        int r = coord.row(index);
        int c = coord.col(index);
        int indexImage = r * ncol * 3 + c * 3;
        if (indexImage < nrow * ncol * 3) {
            image[indexImage + 2] = 255; // red
            image[indexImage + 1] = 0; // green
            image[indexImage + 0] = 0; // blue
        }
        else {
            puts("Error generating image");
        }
    }

    BMPGen gen;
    gen.generateBitmapImage((unsigned char*)&image[0], nrow, ncol, (char*)fname.c_str());
}


void BMPGen::generateBitmapImage(unsigned char* image, int height, int width, char* imageFileName)
{
    int widthInBytes = width * BYTES_PER_PIXEL;

    unsigned char padding[3] = { 0, 0, 0 };
    int paddingSize = (4 - (widthInBytes) % 4) % 4;

    int stride = (widthInBytes)+paddingSize;

    FILE* imageFile = fopen(imageFileName, "wb");

    unsigned char* fileHeader = createBitmapFileHeader(height, stride);
    fwrite(fileHeader, 1, FILE_HEADER_SIZE, imageFile);

    unsigned char* infoHeader = createBitmapInfoHeader(height, width);
    fwrite(infoHeader, 1, INFO_HEADER_SIZE, imageFile);

    int i;
    for (i = 0; i < height; i++) {
        fwrite(image + (i * widthInBytes), BYTES_PER_PIXEL, width, imageFile);
        fwrite(padding, 1, paddingSize, imageFile);
    }

    fclose(imageFile);
}


unsigned char* BMPGen::createBitmapFileHeader(int height, int stride) {
    int fileSize = FILE_HEADER_SIZE + INFO_HEADER_SIZE + (stride * height);

    static unsigned char fileHeader[] = {
        0,0,     /// signature
        0,0,0,0, /// image file size in bytes
        0,0,0,0, /// reserved
        0,0,0,0, /// start of pixel array
    };

    fileHeader[0] = (unsigned char)('B');
    fileHeader[1] = (unsigned char)('M');
    fileHeader[2] = (unsigned char)(fileSize);
    fileHeader[3] = (unsigned char)(fileSize >> 8);
    fileHeader[4] = (unsigned char)(fileSize >> 16);
    fileHeader[5] = (unsigned char)(fileSize >> 24);
    fileHeader[10] = (unsigned char)(FILE_HEADER_SIZE + INFO_HEADER_SIZE);

    return fileHeader;
}

unsigned char* BMPGen::createBitmapInfoHeader(int height, int width)
{
    static unsigned char infoHeader[] = {
        0,0,0,0, /// header size
        0,0,0,0, /// image width
        0,0,0,0, /// image height
        0,0,     /// number of color planes
        0,0,     /// bits per pixel
        0,0,0,0, /// compression
        0,0,0,0, /// image size
        0,0,0,0, /// horizontal resolution
        0,0,0,0, /// vertical resolution
        0,0,0,0, /// colors in color table
        0,0,0,0, /// important color count
    };

    infoHeader[0] = (unsigned char)(INFO_HEADER_SIZE);
    infoHeader[4] = (unsigned char)(width);
    infoHeader[5] = (unsigned char)(width >> 8);
    infoHeader[6] = (unsigned char)(width >> 16);
    infoHeader[7] = (unsigned char)(width >> 24);
    infoHeader[8] = (unsigned char)(height);
    infoHeader[9] = (unsigned char)(height >> 8);
    infoHeader[10] = (unsigned char)(height >> 16);
    infoHeader[11] = (unsigned char)(height >> 24);
    infoHeader[12] = (unsigned char)(1);
    infoHeader[14] = (unsigned char)(BYTES_PER_PIXEL * 8);

    return infoHeader;
}

}
