# Acquire driver for PVCAM

This is an Acquire driver that supports some Teledyne Photometrics cameras using the PVCAM SDK.

## Usage

The intended way to use this library is as part of
the [acquire-imaging](https://github.com/acquire-project/acquire-python)
Python package, which also includes Acquire's runtime as well as other drivers.

That package can be installed using the following command in an environment with Python and `pip`.

```
python -m pip install acquire-imaging
```

### Prerequisites

Before using this driver you must install the [PVCAM SDK](https://www.photometrics.com/support/download/pvcam-sdk).
This driver is built against version 3.10.1.1 of the SDK and you should install a version in the 3.10 family.

### Supported cameras

The following cameras have been tested using this driver.

- [Prime BSI sCMOS](https://www.photometrics.com/products/prime-family/primebsi)

### Supported operating systems

Windows is the primary usage target.
But both Windows and macOS are currently supported for development purposes.

## Development

We welcome contributors. The following will help you get started building the code.

### Environment

Requires

- CMake 3.13+ from its [download page](https://cmake.org/download/), or via
  [chocolatey](https://community.chocolatey.org/packages/cmake) or [homebrew](https://formulae.brew.sh/formula/cmake).
- A C++20 compiler such as [Microsoft Visual Studio](https://visualstudio.microsoft.com/downloads/)
  or [Clang](https://clang.llvm.org/).

### Configure

From the repository root, run the following commands to configure the CMake build files.

```
mkdir build
cd build
cmake ..
```

### Build

After configuration, run the following command to build the library and tests.

```
cmake --build .
```

### Test

After building the default debug configuration, run the following command to run all the tests using CTest.

```
ctest -C Debug
```

You will need all supported cameras connected to the device that you're testing on for all tests to pass.

If you only want to run a subset of the tests you can use CTest's `-R` option to specify a pattern to match against.
For example, if you only want to run the Prime BSI Express tests, run the following command.

```
ctest -C Debug -R prime-bsi
```
