# Design Spec: CMake Best Practice Refactor

**Goal:** Clean up redundant CMake configurations and enable simultaneous compilation of multiple communication channels (e.g., ROS2 + ROSBridge).

## Problem Statement
Current CMake structure has several anti-patterns:
1.  **Redundant Assignments:** `set()` calls are overriding `option()` definitions, making command-line arguments (like `-DBUILD_WITH_CHANNEL_ROS2=OFF`) ineffective.
2.  **Mutually Exclusive Logic:** `src/channel/CMakeLists.txt` uses `if...elseif`, which prevents multiple native channels from being enabled at once.
3.  **Hardcoded Behavior:** ROSBridge is always built regardless of the `BUILD_WITH_CHANNEL_ROSBRIDGE` option.

## Proposed Solution
Refactor the logic to follow "Independent Toggle" patterns with intelligent auto-detection.

### 1. Root `CMakeLists.txt` Cleanup
- Define clean `option()` flags for all channels.
- **Remove all redundant `set()` calls** that override these options.
- The `BUILD_WITH_CHANNEL_AUTO` option will serve as a default-initializer but won't block manual overrides.

### 2. `src/channel/CMakeLists.txt` Logic Refactor
- Change `if...elseif` to independent `if()` blocks.
- If `BUILD_WITH_CHANNEL_AUTO` is `ON`, perform environment detection to enable the appropriate native channel flag *only if not already set*.
- Always respect the specific `BUILD_WITH_CHANNEL_...` flags.

## Design Details

### Option Definitions (Root)
```cmake
option(BUILD_WITH_CHANNEL_AUTO "Auto detect ROS environment" ON)
option(BUILD_WITH_CHANNEL_ROS1 "Build with ROS1 native support" OFF)
option(BUILD_WITH_CHANNEL_ROS2 "Build with ROS2 native support" OFF)
option(BUILD_WITH_CHANNEL_ROSBRIDGE "Build with ROSBridge support" ON)
```

### Dispatch Logic (src/channel)
```cmake
# 1. Handle Auto-Detection
if(BUILD_WITH_CHANNEL_AUTO)
    if(DEFINED ENV{ROS_VERSION})
        if("$ENV{ROS_VERSION}" STREQUAL "1")
            set(BUILD_WITH_CHANNEL_ROS1 ON)
        elseif("$ENV{ROS_VERSION}" STREQUAL "2")
            set(BUILD_WITH_CHANNEL_ROS2 ON)
        endif()
    endif()
endif()

# 2. Independent Build Blocks
if(BUILD_WITH_CHANNEL_ROS1)
    add_subdirectory(ros1)
endif()

if(BUILD_WITH_CHANNEL_ROS2)
    add_subdirectory(ros2)
endif()

if(BUILD_WITH_CHANNEL_ROSBRIDGE)
    add_subdirectory(rosbridge)
endif()
```

## Testing Plan
1.  **Standard Build:** Run `cmake ..` in a ROS2 environment. Verify both `ros2` and `rosbridge` subdirectories are processed.
2.  **Manual Override:** Run `cmake .. -DBUILD_WITH_CHANNEL_ROS2=OFF`. Verify only `rosbridge` is built.
3.  **Cross-Build Simulation:** Run `cmake .. -DBUILD_WITH_CHANNEL_ROS1=ON -DBUILD_WITH_CHANNEL_ROSBRIDGE=ON`. Verify both are enabled.

## Success Criteria
- No redundant `set()` warnings or overrides.
- Users can build any combination of channels via standard CMake flags.
- `BUILD_WITH_CHANNEL_AUTO` correctly initializes flags based on the environment.
