# Change Log

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/), and this project adheres to
[Semantic Versioning](http://semver.org).



## v1.2.0 - 2018-12-7

### Added
 - Added shallow clone for `hero-gcc-toolchain` when `HERO_CI` is set.
 - Removed build folder after installation on every GCC build stage when `HERO_CI` is set.
 - Added Jenkins Pipeline CI.
 - Added CI tests based on `plptest` framework.
 - New command `hero-z-7045-builder -z`: artifact generation for `hero-gcc-toolchain` module.
 - New command `hero-z-7045-builder -c`: CI test execution.
 - Added automatic download of prebuild artifact when `hero-z-7045-builder -A` is executed.

### Changed
- Updated `hero-gcc-toolchain` to v2.0.0.
- Updated `libhero-target` to v1.2.0. Adds HERO timer support and CI tests.
- Updated `hero-openmp-examples` to v1.3.0. Adds CI tests.

### Fixed
- Improved CI clone execution time (Fixed #24).
- Scripts are now based on absolute addresses. Fix for CI tests.
- Minor fix at `README.md`.


## v1.1.0 - 2018-10-02

### Changed
- Updated `hero-openmp-examples` to `v1.2.1`: enabling extensible (double-colon) `run` and `clean`
  Makefile rules and readding the inclusion of `omp.h`; rename `make.inc` into `common/default.mk`
- Update `libhero-target` to `v1.1.0`, fixing the `omp.h` inclusion in the heterogeneous application
  (#22).
- Updated `hero-support` to `v1.0.1`, fixing error reporting in the Zynqlinux SD card deployment
  script.

### Fixed
- Fix the inclusion of `omp.h` in the heterogeneous application (#22).  Remove the dependency of
  `pulp.h` on `libhero-target`.



## v1.0.3 - 2018-09-24

### Fixed
- Updated `pulp-sdk` to `hero-v1.0.2` (bugfix `sdk:archi-host` #32)



## v1.0.2 - 2018-09-19

### Fixed
- Updated `pulp-sdk` to `hero-v1.0.1` (additional fix for #28)
- Updated hero-z-7045-builder. Removed `make all env` from `pulp-sdk` (additional fix for #28)



## v1.0.1 - 2018-09-18

### Added
- FAQ: Add entries on missing UART consumer application and SVM accesses without `tryX`.

### Fixed
- `README.md`: Correct instructions and statements, fix some typos.
- Updated `pulp-sdk` to fix issue #28 (partial fix).

### Changed
- Updated `hero-gcc-toolchain` to v1.0.0.
- Updated `hero-openmp-examples` to v1.1.0, adding the Sobel filter OpenMP example application and
  fixing an include guard and directory creation during application deployment.
- Updated `hero-support` to v1.0.0, the initial release for the ZC706.
- Updated `libhero-target` to v1.0.1, adding API documentation.



## v1.0.0 - 2018-09-14

Initial public release for ZC706
