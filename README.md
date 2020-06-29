# Segway

## Table of Contents

* [About the Project](#about-the-project)
  * [Built With](#built-with)
* [Getting Started](#getting-started)
  * [Prerequisites](#prerequisites)
  * [Installation](#installation)
* [Usage](#usage)
* [Authors](#authors)
* [Acknowledgments](#acknowledgments)


## About The Project

![EasyControl logo](https://gitlab.com/turboillegali/segway/-/blob/master/media/logo_large.png)


In ```Segway``` we implement a control system that stabilize the robot in its
vertical position, which corresponds to the unstable equilibrium state. The
hardware board used to develop the robot is a LEGO MINDSTORM EV3. We later
extend the model from in-place balancing to allow free movement in all
directions. We also developed ```EasyControl``` an Android app with an intuitive
GUI to control the robot easily through a joystick-like controller.

For more information, read the [report](Segway_report.pdf) located in the repo
root.

### Built With

* [ev3dev](https://www.ev3dev.org/)
* [Python](https://www.python.org/)
* [C++](https://www.cplusplus.com/)
* [Kotlin](https://kotlinlang.org/)
* [Android Studio](https://developer.android.com/studio/intro)



## Getting Started

To get a local copy up and running follow these simple steps.

### Prerequisites

The robot need to have ```ev3dev``` installed and to be set up as shown in the
report.

### Installation
 
1. Clone the repo
```sh
git clone https://gitlab.com/turboillegali/gutenberg
```
2. Cross-compile with ```CMake``` and ```Docker``` the contents of the ```cpp```
directory

3. Copy and run the resulting binary on the robot


## Usage

Here's a brief description of each directory in the repo root:

* ```python```: contains both the Python implementations of the control model
* ```cpp```: contains the C++ implementation of the extended control model
* ```app```: contains all the sources of ```EasyControl```

## Authors

* [**Lorenzo Agnolucci**](https://github.com/LorenzoAgnolucci)
* [**Alberto Baldrati**](https://github.com/ABaldrati)
* [**Giovanni Berti**](https://github.com/giovanniberti)


## Acknowledgments
Laboratory of Automatic Control © Course held by Professor [Michele Basso](https://scholar.google.it/citations?user=wa14fi0AAAAJ&hl=it) - Computer Engineering Master Degree @[University of Florence](https://www.unifi.it/changelang-eng.html)
