# Physically-based Simulation 2018 - Course Exercises

## News and Changes

<!-- 20.02.2018 17:30 - Added Exercise 1. -->

Follow the instructions to update your private repository.

## Exercise Overview

Not available yet.
<!-- [Exercise 1: Time Integration](1_canonball/README.md) (Due date: 09.03.2018 09:00)   -->

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