# RTOSim Plugin

## Requirements

 * RTOSim (Bitbucket fork)
 * RTB::Concurrency
 * RTB::Filter
 * Qualisys_cpp_sdk
 * XSD
 * Xerces

## Build

Build with the regular:

```shell
mkdir build && cd build
cmake ..
cmake --build .
```

### Options

Set the flag `-DUNFILT=ON` to enable the unfiltered ground reaction forces compile flag.
