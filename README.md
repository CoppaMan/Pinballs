# Physically-based Simulation 2018 - Course Exercises

## News and Changes

25.09.2018 - Added Exercise 1. Please update your forked repository first.

19.09.2018 - Follow the instructions to update your private repository.

## Exercise Overview

<!-- Not available yet. -->
[Exercise 1: Time Integration](ex1.pdf) (Due date: 02.09.2018 23:59)

## Installation

### Git and CMAKE
Before we can begin, you must have Git running, a distributed revision control system which you need to handin your assignments as well as keeping track of your code changes. We refer you to the online [Pro Git book](https://git-scm.com/book/en/v2) for more information. There you will also find [instructions](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git]) on how to to install it. On windows we suggest using [git for windows](https://git-for-windows.github.io/).

CMake is the system this framework uses for cross-platform builds. If you are using Linux or macOS, I recommend installing it with a package manager instead of the CMake download page. E.g. on Debian/Ubuntu:
```
sudo apt-get install cmake
```
or with MacPorts on macOS:
```
sudo port install cmake.
```
On Windows you can download it from:
[https://cmake.org/download/](https://cmake.org/download/)

### Note for linux users

Many linux distributions do not include `gcc` and the basic development tools in their default installation. On Ubuntu, you need to install the following packages:

```
sudo apt-get install build-essential
sudo apt-get install libx11-dev
sudo apt-get install mesa-common-dev libgl1-mesa-dev libglu1-mesa-dev
sudo apt-get install libxrandr-dev
sudo apt-get install libxi-dev
sudo apt-get install libxmu-dev
sudo apt-get install libblas-dev
sudo apt-get install libxinerama-dev
sudo apt-get install libxcursor-dev
```

If you are using linux with a virtual machine on Windows, it is recommended to use Visual Studio instead.

### Note for Windows users

`libigl` supports the Microsoft Visual Studio 2015 compiler and later, in 64bit mode. You can download them for free from [ETH-Microsoft webstore](https://e5.onthehub.com/WebStore/ProductsByMajorVersionList.aspx?cmi_cs=1&cmi_mnuMain=bdba23cf-e05e-e011-971f-0030487d8897&ws=5664fddb-836f-e011-971f-0030487d8897&vsro=8).


### Cloning the Exercise Repository
Before you are able to clone your private exercise repository, you need to have an active [Gitlabvis](https://gitlab.vis.ethz.ch/) account. Then you can [fork](https://docs.gitlab.com/ee/gitlab-basics/fork-project.html) this project to create your own private online repository.

In the next step you need to clone it to your local hard drive:
```
git clone --recurse-submodules https://gitlab.vis.ethz.ch/'Your_Git_Username'/PBS18-Exercises.git
```
'Your_Git_Username' needs to be replaced accordingly. This can take a moment.

Next, cd into the newly created folder, and run the following commands inside the relevant subfolder to setup the build folder:
```
cd PBS18-Exercises; mkdir build
cd build
cmake ..
```
On Windows use the CMAKE gui with the buttons Configure and Generate.

Compile and run the executable, e.g.:
```
make && ./0_dummy
```
Or use your favorite IDE.

To update your forked repository, check this page: [how-do-i-update-a-github-forked-repository](https://stackoverflow.com/questions/7244321/how-do-i-update-a-github-forked-repository)

Basically, you are required to add our repository as a remote to your own one (just once):
```
git remote add upstream https://gitlab.vis.ethz.ch/kimby/PBS18-Exercises.git
```
Then, fetch updates from it (should be done again for new updates):
```
git fetch upstream
```
Lastly, move to your `master` branch and merge updates:
```
git merge upstream/master
```