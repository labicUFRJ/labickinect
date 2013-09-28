LabicKinect
=============================

LabicKinect is a software capable of reconstructing 3D maps of environments with a Kinect, being developed as a graduation project at UFRJ. Please notice the software is still under early development and it not stable at all. Please read the support terms before using it.

- Developed at the Laboratory Computational Intelligence - NCE/UFRJ ([http://www.labic.nce.ufrj.br](http://www.labic.nce.ufrj.br))
- Project and code: Mario Cecchi ([http://meriw.com](http://meriw.com))
- Orientation: Adriano Joaquim de Oliveira Cruz ([http://equipe.nce.ufrj.br/adriano](http://equipe.nce.ufrj.br/adriano))

- - -

# Requirements
- [CMake](http://www.cmake.org/) >= 2.8
- [libfreenect](https://github.com/OpenKinect/libfreenect) >= 0.1.2
- [OpenCV](http://opencv.org) >= 2.4.6
- [libPCL](http://pointclouds.org) >= 1.7

- - -

# Compiling
## Dependencies installation
Please follow to the installation of each library listed below before continuing.
### Ubuntu 12.04.3
#### CMake
	sudo apt-get install cmake
#### libPCL
	sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
	sudo apt-get update
	sudo apt-get install libpcl-all
#### OpenCV
	version="$(wget -q -O - http://sourceforge.net/projects/opencvlibrary/files/opencv-unix | egrep -m1 -o '\"[0-9](\.[0-9])+' | cut -c2-)"
	echo "Installing OpenCV" $version
	mkdir OpenCV
	cd OpenCV
	echo "Removing any pre-installed ffmpeg and x264"
	sudo apt-get -qq remove ffmpeg x264 libx264-dev
	echo "Installing Dependenices"
	sudo apt-get -qq install libopencv-dev build-essential checkinstall cmake pkg-config yasm libtiff4-dev libjpeg-dev libjasper-dev libavcodec-dev libavformat-dev libswscale-dev libdc1394-22-dev libxine-dev libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev libv4l-dev python-dev python-numpy libtbb-dev libqt4-dev libgtk2.0-dev libfaac-dev libmp3lame-dev libopencore-amrnb-dev libopencore-amrwb-dev libtheora-dev libvorbis-dev libxvidcore-dev x264 v4l-utils ffmpeg
	echo "Downloading OpenCV" $version
	wget -O OpenCV-$version.tar.gz http://sourceforge.net/projects/opencvlibrary/files/opencv-unix/$version/opencv-"$version".tar.gz/download
	echo "Installing OpenCV" $version
	tar -xvf OpenCV-$version.tar.gz
	cd opencv-$version
	mkdir build
	cd build
	cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D WITH_TBB=ON -D WITH_V4L=ON -D INSTALL_C_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_QT=ON -D WITH_OPENGL=ON ..
	make -j8
	sudo make install
	sudo sh -c 'echo "/usr/local/lib" > /etc/ld.so.conf.d/opencv.conf'
	sudo ldconfig
	echo "OpenCV" $version "ready to be used"
#### libfreenect
	sudo apt-get install git-core cmake libglut3-dev pkg-config build-essential libxmu-dev libxi-dev libusb-1.0-0-dev
	git clone git://github.com/OpenKinect/libfreenect.git
	cd libfreenect
	mkdir build
	cd build
	cmake ..
	make
	sudo make install
	sudo ldconfig /usr/local/lib64/

You can test if the Kinect is working by running `sudo glview`.
## Compiling sources
	git clone https://bitbucket.org/macecchi/labickinect.git
	cd labickinect
	mkdir build
	cd build
	cmake ..
	make
If you are using OS X, you can generate project files for Xcode:
	cmake -G Xcode ..
You can also generate project files for Eclipse, but please create a new directory outside of the source folder, like a folder `labickinect-eclipse` at the same path of the `labickinect`.
	mkdir labickinect-eclipse
	cd labickinect-eclipse
	cmake -G "Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug ../labickinect


- - -

# Running
You can run the project by calling `./labickinect` from the terminal. Make sure your Kinect is connected. Instructions on how to use the program will be provided.

- - -

# Support
The code has been tested only on **Ubuntu 12.04.3** and partly on **OS X 10.8**.

There is **NO** support, as the software is still in an early stage of development and its only purpose at the moment is to serve the research laboratory. However, feel free to submit bug reports, feature requests, etc.

If you want, you can contact me directly though e-mail at macecchi@gmail.com.

- - -

# Licensing

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

- - -

# References
This project is based on Peter Henry, Michael Krainin, Evan Herbst, Xiaofeng Ren and Dieter Fox, "RGB-D mapping: Using Kinect-style depth cameras for dense 3D modeling of indoor environments" *(The International Journal of Robotics Research, 2012)*
