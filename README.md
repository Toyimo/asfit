# ALGLIB Spline Fitting Library

ALGLIB Spline Fitting Library

## File Structure  

```bash
├── CMakeLists.txt
├── README.md
├── alglib                    # ALGLIB src files
├── alglib_spline_fitting.cpp # spline fitting use alglib.spline1dfit
├── alglib_spline_fitting.h   # include file
├── demo.cpp                  # a demo executable file
├── demo.py                   # convert data to geojson
└── example                   # how to use dll library
    ├── CMakeLists.txt
    ├── demo.cpp
    ├── include
    └── lib
```

## How to use 

1. Create Folder and Excute CMAKE Command

```bash
mkdir build
cd build
cmake ..
make & make install
```

2. Copy the `include` and `lib` folder from `install` to `example`

```bash
cp -r install/* example
```

3. Jump into `example` folder and excute command

```bash
cd example
mkdir build
cd build
cmake ..
make
```