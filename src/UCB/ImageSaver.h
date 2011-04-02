/*
 * ImageSaver.h
 *
 *  Created on: Oct 23, 2008
 *      Author: njoubert
 */

#ifndef UCBIMAGESAVER_H_
#define UCBIMAGESAVER_H_

#include <sstream>
#include <iomanip>
#include "Image.h"
#include "UCBglobal.h"

namespace UCB {

	using namespace std;

    #define BPP 24

    class ImageSaver {
    public:

        /**
         * Creates an ImageSaver that will write OpenGL window captures
         * to the specified directory with files names prefix000i.bmp
         */
        ImageSaver(std::string directory, std::string prefix);

        ~ImageSaver();

        /**
         * Call this to write a frame to disk. Supply the current window size.
         */
        void saveFrame();

    private:
        int _frameCount;
        string _imgOutDir;
        string _prefix;
    };

}
#endif /* UCBIMAGESAVER_H_ */
