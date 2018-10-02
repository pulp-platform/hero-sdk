# Change Log

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/), and this project adheres to
[Semantic Versioning](http://semver.org).

## v1.1.0 - 2018-10-02

### Changed
- Update `hero-openmp-examples` to `v1.2.1`: enabling extensible (double-colon) `run` and `clean`
  Makefile rules and readding the inclusion of `omp.h`; rename `make.inc` into `common/default.mk`
- Update `libhero-target` to `v1.1.0`, fixing the `omp.h` inclusion in the heterogeneous application
  (#22).
- Update `hero-support` to `v1.0.1`, fixing error reporting in the Zynqlinux SD card deployment
  script.

### Fixed
- Fix the inclusion of `omp.h` in the heterogeneous application (#22).  Remove the dependency of
  `pulp.h` on `libhero-target`.

## v1.0.3 - 2018-09-24

### Fixed
- Update `pulp-sdk` to `hero-v1.0.2` (bugfix `sdk:archi-host` #32)

## v1.0.2 - 2018-09-19

### Fixed
- Update `pulp-sdk` to `hero-v1.0.1` (additional fix for #28)
- Update hero-z-7045-builder. Removed `make all env` from `pulp-sdk` (additional fix for #28)

## v1.0.1 - 2018-09-18

### Added
- FAQ: Add entries on missing UART consumer application and SVM accesses without `tryX`.

### Fixed
- ReadMe: Correct instructions and statements, fix some typos.
- Update `pulp-sdk` to fix issue #28 (partial fix).

### Changed
- Update `hero-gcc-toolchain` to v1.0.0.
- Update `hero-openmp-examples` to v1.1.0, adding the Sobel filter OpenMP example application and
  fixing an include guard and directory creation during application deployment.
- Update `hero-support` to v1.0.0, the initial release for the ZC706.
- Update `libhero-target` to v1.0.1, adding API documentation.

## v1.0.0 - 2018-09-14

Initial public release for ZC706
