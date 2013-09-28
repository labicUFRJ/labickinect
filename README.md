LabicKinect
=============================

LabicKinect is a software capable of reconstructing 3D maps of environments with a Kinect, being developed as a graduation project. Please notice the software is still under early stages and it not stable at all. Please read the support terms before using it.

- Developed at the Laboratory Computational Intelligence - NCE/UFRJ ([http://www.labic.nce.ufrj.br])
- Project and code: Mario Cecchi ([http://meriw.com])
- Orientation: Adriano Joaquim de Oliveira Cruz ([http://equipe.nce.ufrj.br/adriano])

- - -

# Requirements
- [CMake](http://www.cmake.org/) >= 2.8
- [libfreenect](https://github.com/OpenKinect/libfreenect) >= 0.1.2
- [OpenCV](http://opencv.org) >= 2.4.6
- [libPCL](http://pointclouds.org) >= 1.7

- - -

# Compiling
## Dependencies installation
Please follow to the installation of each library listed above before continuing.

## Compiling source
	git clone https://macecchi@bitbucket.org/macecchi/labickinect.git
	cd labickinect
	mkdir build
	cd build
	cmake ..
	make

- - -

# Running
You can run the project by calling `./labickinect` from the terminal. Make sure your Kinect is connected. Instructions on how to use the program will be provided.

- - -

# Support
There is **NO** support, as the software is still in an early stage of development and its only purpose at the moment is to serve the research laboratory. However, feel free to submit bug reports, feature requests, etc. If you want, you can contact me directly though e-mail at macecchi@gmail.com.

The code has been tested on **Ubuntu 12.04.3** and partly on **OS X 10.8**. I've never tried to run it on Windows.

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


# References
This project is based on Peter Henry, Michael Krainin, Evan Herbst, Xiaofeng Ren and Dieter Fox, "**RGB-D mapping: Using Kinect-style depth cameras for dense 3D modeling of indoor environments**" *(The International Journal of Robotics Research, 2012)*
