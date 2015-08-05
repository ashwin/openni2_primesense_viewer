Simple program to demonstrate how to build and read from a Primesense camera using OpenNI2.

Requirements:

First make sure you install OpenNI2 library.

OpenCV is used for displaying the depth and color (RGB) images.
It can installed easily:

```
$ sudo apt install libopencv-dev
```

To compile:

```
$ mkdir build
$ cd build
$ cmake ..
$ make
```

To run, first connect the Primensense camera and then run:

```
$ cd bin
$ ./openni2_primesense_viewer
```

- Ashwin Nanjappa
