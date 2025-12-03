3# Git Tag Documentation

This document describes the tagging strategy for the Heartprinter project.

## Tagging Strategy

The Heartprinter project uses **Semantic Versioning** (SemVer) for releases, following the format:

```
vMAJOR.MINOR.PATCH
```

- **MAJOR**: Incremented for incompatible API/hardware changes or major system redesigns
- **MINOR**: Incremented for new features added in a backward-compatible manner
- **PATCH**: Incremented for backward-compatible bug fixes

### Tag Format

All release tags follow this format:
- **Production releases**: `vX.Y.Z` (e.g., `v1.0.0`, `v1.2.3`)
- **Pre-release versions**: `vX.Y.Z-alpha`, `vX.Y.Z-beta`, `vX.Y.Z-rc.N`
- **Milestone tags**: Descriptive names for major development milestones

## Current Tags

### Production Tags

#### `v0.9.0` - Release Candidate
**Date**: 2025-12-03
**Branch**: main
**Description**: Feature-complete release candidate ready for final testing and validation.

**Key Features**:
- Complete state machine implementation
- Dynamixel motor control (3-motor cable-driven parallel positioning)
- Northern Digital trakSTAR 3D position tracking
- NI-DAQmx load cell monitoring
- Python GUI with shared memory communication
- Real-time control loop with safety checks
- Automatic calibration routines
- Comprehensive logging system
- Cross-platform support (Windows/Linux)

#### `v1.0.0` - Production Ready Release
**Date**: TBD (Planned after testing v0.9.0)
**Description**: First stable production release after validation of v0.9.0 release candidate.

**Planned additions**:
- Enhanced documentation
- Code cleanup and optimization
- Validated performance metrics
- Production-ready configuration
- **Ubuntu/Linux testing for NI-DAQmx compatibility verification**

---

### Historical Milestone Tags

These tags mark significant development milestones during the project evolution:

#### `trackstar_integration_complete`
**Description**: Completed integration of Northern Digital trakSTAR electromagnetic tracking system

**Key Changes**:
- Full trakSTAR API integration
- Sensor calibration routines
- Position tracking and monitoring

#### `state_machine_start`
**Description**: Initial implementation of the state machine architecture

**Key Changes**:
- Basic state machine structure
- State transition logic
- System states: START, INIT, READY, MOVE, ERROR, CLEANUP, END

#### `begin_trackstar_integration`
**Description**: Started trakSTAR hardware integration work

#### `Windows_init`
**Description**: Windows platform initialization and setup

#### `WIN_W_DX_TS`
**Description**: Windows build with Dynamixel and TrackStar integration

#### `PC_Viz`
**Description**: Early PC visualization/GUI work

#### `InitTestC++`
**Description**: Initial C++ testing and setup

---

## Creating New Tags

### For Production Releases

When creating a production release, use the following process:

```bash
# Create an annotated tag
git tag -a v1.0.0 -m "Production release v1.0.0

- Complete state machine architecture
- Full hardware integration (motors, trakSTAR, DAQ)
- Comprehensive documentation
- Cross-platform build system"

# Push the tag to remote
git push origin v1.0.0
```

### For Milestones

For development milestones, use descriptive tags:

```bash
# Create a lightweight or annotated tag
git tag milestone_name -m "Description of milestone"

# Push to remote
git push origin milestone_name
```

---

## Tag Guidelines

### When to Create a Tag

**Create a new tag when**:
- Completing a major feature or system component
- Reaching a stable release point
- Before major refactoring or architectural changes
- After completing comprehensive documentation updates

**Don't create a tag for**:
- Minor bug fixes during development
- Work-in-progress commits
- Experimental features

### Tag Naming Conventions

- Use **lowercase** for milestone tags (e.g., `feature_complete`, `beta_release`)
- Use **vX.Y.Z format** for version tags (e.g., `v1.0.0`, `v2.1.3`)
- Be **descriptive** but **concise** (e.g., `motor_calibration_complete` not `added_motor_calibration_and_some_fixes`)
- Use **underscores** for multi-word milestone tags

### Tag Messages

Always include a message with tags:
- **Summary**: One-line description
- **Key changes**: Bullet points of major changes
- **Breaking changes**: Note any incompatibilities (if applicable)
- **Documentation**: Link to relevant docs or READMEs

---

## Version History

| Version | Date | Description | Branch |
|---------|------|-------------|--------|
| v0.9.0 | 2025-12-03 | Release candidate - Feature complete | main |
| v1.0.0 | TBD | Production release (planned) | main |

---

## Deprecation Policy

When deprecating features or APIs:
1. Announce in release notes
2. Keep deprecated code for at least one MINOR version
3. Add warnings/logs for deprecated functionality
4. Document migration path in CHANGELOG.md

---

## Related Documentation

- [README.md](README.md) - Project overview
- [code/Cpp/README.md](code/Cpp/README.md) - C++ system documentation
- [code/Python/README.md](code/Python/README.md) - Python GUI documentation
- [code/Cpp/docs/](code/Cpp/docs/) - Detailed technical documentation
