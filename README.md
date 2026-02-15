# NTHU-Route 2.0 — VLSI Global Router

[![Build](https://img.shields.io/badge/build-CMake-blue)](CMakeLists.txt)
[![C++14](https://img.shields.io/badge/C%2B%2B-14-blue.svg)](https://isocpp.org/)
[![License](https://img.shields.io/badge/license-see%20LICENSE-green)](LICENSE)

NTHU-Route 2.0 is a fast and stable global router for VLSI design. It won **1st place** at the [ISPD 2008 Global Routing Contest](http://archive.sigda.org/ispd2008/contests/ispd08rc.html), generating the best solutions for 11 of 16 benchmarks among all participating routers.

## Overview

The router implements several key enhancements over the original NTHU-Route:

1. **History-based cost function** for congestion-aware routing
2. **Ordering methods** for congested region identification and rip-up & reroute
3. **Implementation-oriented techniques** for improved runtime and solution quality

For detailed algorithm descriptions, see the [Algorithm Documentation](docs/algorithms.md).

## Authors

- **Original:** Yen-Jung Chang, Yu-Ting Lee, Tsung-Hsien Lee, Jhih-Rong Gao, Pei-Ci Wu
- **Adviser:** Ting-Chi Wang (tcwang@cs.nthu.edu.tw)
- **Modern C++ port:** Florian Prud'homme

Contact: NTHU.Route@gmail.com
[Project Homepage](http://www.cs.nthu.edu.tw/~tcwang/nthuroute/)

## Prerequisites

- **C++ compiler** with C++14 support (GCC ≥ 7, Clang ≥ 5)
- **CMake** ≥ 3.14
- **Boost** (header-only libraries: multi_array, heap, range, functional)
- **zlib** (for gzip file support)
- **pthread**

### Install dependencies on Ubuntu / Debian

```bash
sudo apt-get install build-essential cmake libboost-all-dev zlib1g-dev
```

## Building

```bash
# Configure
cmake -B build -DCMAKE_BUILD_TYPE=Release

# Build
cmake --build build -j$(nproc)
```

The resulting binary `NthuRoute` will be in the `build/` directory.

### Build types

| Type | Flags | Use case |
|------|-------|----------|
| `Release` | `-O3 -DNDEBUG` | Production / benchmarking |
| `Debug` | `-g -O0` | Development / debugging |

```bash
# Debug build
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j$(nproc)
```

### Running Tests

Tests use [Google Test](https://github.com/google/googletest). Install it (`sudo apt-get install libgtest-dev` on Ubuntu), then:

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j$(nproc)
cd build && ctest --output-on-failure
```

### Install (optional)

```bash
sudo cmake --install build
```

## Usage

### Command-line application

```bash
./build/NthuRoute --help
```

```bash
./build/NthuRoute \
  --input=adaptec1.capo70.3d.35.50.90.gr \
  --output=output.txt \
  --p2-max-iteration=150 \
  --p2-init-box-size=25 \
  --p2-box-expand-size=1 \
  --overflow-threshold=0 \
  --p3-max-iteration=20 \
  --p3-init-box-size=10 \
  --p3-box-expand-size=15 \
  --monotonic-routing=0
```

> **Note:** The FLUTE lookup tables (`POWV9.dat` and `POST9.dat`) must be in the working directory. CMake copies them to the build directory automatically.

### Options

| Option | Description |
|--------|-------------|
| `--input=FILE` | Input testcase file (required) |
| `--output=FILE` | Output result file (required) |
| `--p2-init-box-size=N` | Initial bounding-box size in main stage |
| `--p2-box-expand-size=N` | Bounding-box expanding size in main stage |
| `--p2-max-iteration=N` | Maximum iterations in main stage |
| `--p3-init-box-size=N` | Initial bounding-box size in refinement stage |
| `--p3-box-expand-size=N` | Bounding-box expanding size in refinement stage |
| `--p3-max-iteration=N` | Maximum iterations in refinement stage |
| `--overflow-threshold=N` | Overflow threshold in main stage |
| `--monotonic-routing={0,1}` | Enable/disable monotonic routing per iteration |
| `--help`, `-h` | Show usage information and exit |
| `--version`, `-v` | Show version information and exit |

### Library usage

```cpp
#include "src/router/Route.h"

NTHUR::Route router;
NTHUR::RoutingRegion rr(3, 3, 2);
rr.setVerticalCapacity(1, 2);
rr.setHorizontalCapacity(0, 2);
rr.setNetNumber(1);
rr.beginAddANet("A", 0, 2, 1);
rr.addPin(0, 0, 1);
rr.addPin(2, 0, 1);
rr.endAddANet();

NTHUR::OutputGeneration output(router.process(rr, spdlog::level::trace));
NTHUR::OutputGeneration::Comb comb(output.combAllNet());

for (const std::vector<NTHUR::Segment3d>& v : comb) {
    for (const NTHUR::Segment3d& s : v) {
        // Use s.first and s.last for segment endpoints
    }
}
```

See the [ISPD 2008 Global Routing Contest](http://archive.sigda.org/ispd2008/contests/ispd08rc.html) for input format details.

## VS Code Integration

The CMake build generates `compile_commands.json` automatically, enabling IntelliSense and clang-based tooling. Recommended VS Code extensions:

- [CMake Tools](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cmake-tools)
- [C/C++](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools)
- [clangd](https://marketplace.visualstudio.com/items?itemName=llvm-vs-code-extensions.vscode-clangd)

## Linting and Code Style

The project includes configuration for clang-tidy and clang-format:

```bash
# Run clang-format on source files
find src -name '*.cpp' -o -name '*.h' | xargs clang-format --dry-run

# Run clang-tidy (requires compile_commands.json from CMake build)
clang-tidy -p build src/router/Main.cpp
```

- **`.clang-format`** — Based on Google C++ style (4-space indent, 120 column limit)
- **`.clang-tidy`** — Enables bugprone, modernize, performance, and readability checks

## Documentation

- [Algorithm Documentation](docs/algorithms.md) — Detailed descriptions of routing algorithms with diagrams

## Related Publications

- Yen-Jung Chang, Yu-Ting Lee, and Ting-Chi Wang, "[NTHU-Route 2.0: A Fast and Stable Global Router](http://vlsicdb.cs.nthu.edu.tw/~lab221/nthuroute/NTHU-Route2.0.pdf)," ICCAD 2008, pp. 338–343.
- Jhih-Rong Gao, Pei-Ci Wu, and Ting-Chi Wang, "[A New Global Router for Modern Designs](http://vlsicdb.cs.nthu.edu.tw/~lab221/nthuroute/NTHU-Route.pdf)," ASP-DAC 2008, pp. 232–237.
- Tsung-Hsien Lee and Ting-Chi Wang, "[Congestion-Constrained Layer Assignment for Via Minimization in Global Routing](http://vlsicdb.cs.nthu.edu.tw/~lab221/nthuroute/04603083.pdf)," IEEE TCAD, September 2008, pp. 1643–1656.
- T.-H. Lee and T.-C. Wang, "[Robust Layer Assignment for Via Optimization in Multi-layer Global Routing](http://vlsicdb.cs.nthu.edu.tw/~lab221/nthuroute/Layer_Assignment.pdf)," ISPD 2009, pp. 159–166.

## Contributing

Contributions are welcome! Please follow these guidelines:

1. Fork the repository and create a feature branch.
2. Follow the coding style defined in `.clang-format`.
3. Ensure the code compiles without warnings (`cmake --build build`).
4. Run `clang-tidy` on changed files.
5. Submit a pull request with a clear description of changes.

## License

See [LICENSE](LICENSE) for details.
