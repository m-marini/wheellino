# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/en/1.0.0/)
and this project adheres to [Semantic Versioning](http://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Changed

- Issue #91: Change LED status flashing
- Issue #93: Switch on LED utill start

## [0.7.0] 2023-11-12

### Added

- Issue #78: Add friction test
- Issue #84: Add battery level icons
- Issue #86: Add proxy sensor noise filter
- Issue #88: Fix status send frequence on move (TCS review)

### Changed

- Issue #75: Change motion by mixing rotation and movement
- Issue #80: Change stiction and friction test

### Removed

- Issue #82: Remove arduino one board code

## [0.6.1] 2023-10-20

### Added

- Issue #55: Add configure proxy scan interval
- Issue #69: Add physic measures

### Changed

- Issue #71: Change measures with power ramp up and down

### Fixed

- Issue #61: Bug Wifi post config dump
- Issue #64: API get networks list often run in Guru meditation
- Issue #67: Yaw drift
- Issue #73: Fix report with start and stop test records

## [0.6.0] 2023-08-22

### Added

- Issue #47: WheellyESP32

### Changed

- Issue #36: Change motor controller to manage stiction
- Issue #42: Merge filter and speedmeter
- Issue #44: Add power acceleration limits
- Issue #46: New larger base, contact bars and control board

### Fixed

- Issue #38: Error at cl command
- Issue #40: Error parsing cm command

## [0.4.1] 2023-02-09

### Added

- Issue #32: Limit robot angular speed

### Changed

- Issue #34: Change WheellyMeasures to run ramp power measures

## [0.4.0] 2023-02-07

### Added

- Issue #30: Set contact sensors thresholds

## [0.3.2] 2023-01-25

### Fixed

- Isssue #27: Configuration command error

## [0.3.1] 2023-01-20

### Changed

- Issue #22: Change WheellyMeasures
- Issue #24: Change motor power supply and add controller configurations

## [0.3.0] 2023-01-08

### Changed

- Issue #19: Minimal data conversion

## [0.2.5] 2023-01-08

### Added

- Issue #16: Add movement check up
- Issue #15: Bad gyroscope measure moving

## [0.2.4] 2022-09-12

### Added

- Issue #13: Add GET network configuration api

## [0.2.3] 2022-09-12

### Added

- Issue #11: status led for default access point

## [0.2.2] 2022-08-14

### Fixed

- Issue #9: MPU failure bug

## [0.2.1] 2022-07-16

### Fixed

- Issue #7: Standard reference system

## [0.2.0] 2022-07-15

### Changed

- Issue #5: Changed startup led sequence and output wifi IP Address

## [0.1.2] 2022-06-17

### Added

- Issue #1: Release 0.1.2
