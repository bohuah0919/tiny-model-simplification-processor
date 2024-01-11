# Tiny Model Simplification Processor

## Table of Contents

- [Introduction](#introduction)
- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

## Introduction

This is a tiny and effective model simplification processor, which can simplify a model with 870,000 triangles in 6 minutes.
This project use libigl package as frame. 
Algorithm implementation is based on this paper:
Gariand and Heckbert. Surface Simplification Using Quadric Error Metrics. Siggraph 1997.
http://mgarland.org/files/papers/quadrics.pdf

## Features

input model file type supports off/obj/ply.
output model file type supports off/obj.

## Compile
Compile this project using the standard cmake routine:

```bash
mkdir build
cd build
cmake ..
make
```

## Run
From the build directory:
./processor


## Tutorial

Here provides a guide with pictures:

