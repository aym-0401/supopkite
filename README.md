# Supopkite

#### A student's attempt for the real-time 3D tracking of a kite sail in C++, using OpenCV and Eigen



For some years now, companies such as [Beyond the Sea](http://beyond-the-sea.com) have been developping systems to adapt large-scale kite sails to boats bigger than conventional kitesurfs. Such systems aim at reducing the fuel needed to move a boat.

In these prototypes, the control of the kite sail's position, velocity and orientation is based on electronic sensors, physically attached to the kite and remotely sending the information to the computer hardware.

During seven weeks, we formed a group of three students attempting to build from scratch a computer vision prototype for the real-time 3D tracking of a kite sail, aiming to replace the inboard sensors of the sail by a centralized tracking camera system. Such prototype could be an alternative technique to optimize the controlled-behaviour of the kite sail.

This repository submits the C++ code we produced, mainly based on the OpenCV library. The tracking parameters of the code were tuned using a dataset of records of our own kite in various conditions of luminosity. In the end, the code is not mature enough yet as the different functions are not merged yet, but it is an interesting move forward part of a bigger work that includes a GoPro/Raspberry prototype.

## Build

Installation requirements:

- CMake and a C++ compiler
- The OpenCV library
- The Eigen C++ library

```c
git clone https://github.com/cybersupop/supopkite.git
cd supopkite
mkdir build && cd build
cmake ..
make
```

If everything goes well, the executable `./supopkite`is produced.

## The code: usage and short explanations

We attempted to retrieve the position and orientation of a kite sail in the field of view of a camera. For now the code only takes video files as arguments. There are three different modes:

- The `-tracking` mode detects the kite in the video. The user must locate the zone of first detection. Every frame is calculated according to the position in the previous one by detecting closed-contours that features convex curves.

  ```
  ./supopkite ../ressources/test_tracking.mp4 -tracking
  ```

  ​	

- The `-orientation` mode retrieves the orientation of our kite appended with orange colour patches. Knowing the 3D repartition of the patches in space, detecting their position allows to compute the orientation of the kite. The user does not have to locate the zone of first detection.

  ```
  ./supopkite ../ressources/test_orientation.mp4 -orientation
  ```

  

- The `-newdata` mode projects a 3D model of the kitesail (.stl) in order to find the best match, i.e. the orientation of the 3D model that best matches the observed kite. Matching is supposed to be computed using a cross-correlation product in the Fourier domain, but this part of the code is unfinished.

  ```
  ./supopkite ../ressources/3Dkite.stl -newdata
  ```

## License & dependencies

`Created by Aymeric HALÉ, Charlotte MILLET & François OLLITRAULT.`

```
// Created by Aymeric HALÉ, Charlotte MILLET & François OLLITRAULT.
// We give our explicit authorization to use, distribute and modify this code.
// Some portions of the code are authorized modifications of other works indicated
// by their copyrights.
```

The project uses OpenCV and [Eigen](https://eigen.tuxfamily.org) under open source licenses. Parts of the [KissFFT](https://github.com/mborgerding/kissfft) open source library is directly copied within legal rights relying on the BSD clause, which original license can be found in the folder `ext/kissfft`.

The code that manages the .stl projection to 2D images is a slight modification of the project [STL-Renderer](https://github.com/glaba/STL-Renderer), by Charles H. Zega and Saransh Sinha, who gave explicit permission to use, copy and modify their code. I adapted their code to C++ compilation. Their copyright notice is included whenever appropriate.

Several tutorials were useful to produce our code, for instance:

- [Tracking using the Kalman filter](https://github.com/Myzhar/simple-opencv-kalman-tracker) by Myzhar.
- [Tracking color points in openCV](https://www.f-legrand.fr/scidoc/docmml/opencv/python/suivicolor/suivicolor.html) by Frédéric Legrand.

## Contact

You're welcome to contact us if you need further informations about the code or the project :

- [aymeric.hale@institutoptique.fr](mailto:aymeric.hale@institutoptique.fr)
- [charlotte.millet@institutoptique.fr](mailto:charlotte.millet@institutoptique.fr)
- [francois.ollitrault@institutoptique.fr](mailto:francois.ollitrault@institutoptique.fr)

