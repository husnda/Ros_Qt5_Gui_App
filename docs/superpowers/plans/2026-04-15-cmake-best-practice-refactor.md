# CMake Best Practice Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Clean up redundant CMake configurations and enable simultaneous compilation of multiple communication channels.

**Architecture:** Refactor `option()` flags and build dispatch logic to support independent toggles with intelligent defaults.

**Tech Stack:** Modern CMake.

---

### Task 1: Root CMakeLists.txt Cleanup

**Files:**
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Refactor options and remove redundant set calls**

```cmake
# CMakeLists.txt

# Replace lines 33-39 with:
option(BUILD_WITH_CHANNEL_AUTO "Auto detect ROS environment" ON)
option(BUILD_WITH_CHANNEL_ROS1 "Build with ROS1 native support" OFF)
option(BUILD_WITH_CHANNEL_ROS2 "Build with ROS2 native support" OFF)
option(BUILD_WITH_CHANNEL_ROSBRIDGE "Build with ROSBridge support" ON)
option(BUILD_WITH_TEST "Build tests" OFF)
```

- [ ] **Step 2: Commit**

```bash
git add CMakeLists.txt
git commit -m "refactor(cmake): cleanup root options and remove redundant overrides"
```

---

### Task 2: src/channel/CMakeLists.txt Logic Refactor

**Files:**
- Modify: `src/channel/CMakeLists.txt`

- [ ] **Step 1: Refactor dispatch logic**

```cmake
# src/channel/CMakeLists.txt

# Replace lines 4-22 with:
if(BUILD_WITH_CHANNEL_AUTO)
    if(DEFINED ENV{ROS_VERSION})
        if("$ENV{ROS_VERSION}" STREQUAL "1")
            set(BUILD_WITH_CHANNEL_ROS1 ON)
            message("Auto-detected ROS1 environment")
        elseif("$ENV{ROS_VERSION}" STREQUAL "2")
            set(BUILD_WITH_CHANNEL_ROS2 ON)
            message("Auto-detected ROS2 environment")
        endif()
    endif()
endif()

if(BUILD_WITH_CHANNEL_ROS1)
    message("Building ROS1 channel")
    add_subdirectory(ros1)
endif()

if(BUILD_WITH_CHANNEL_ROS2)
    message("Building ROS2 channel")
    add_subdirectory(ros2)
endif()

if(BUILD_WITH_CHANNEL_ROSBRIDGE)
    message("Building ROSBridge channel")
    add_subdirectory(rosbridge)
endif()
```

- [ ] **Step 2: Commit**

```bash
git add src/channel/CMakeLists.txt
git commit -m "refactor(cmake): enable simultaneous channel builds with independent toggle logic"
```

---

### Task 3: Verification

**Files:**
- Verify: `build/`

- [ ] **Step 1: Verify auto-detection (ROS2)**

Run: `cd build && cmake .. && ls lib/libchannel_ros2.so lib/libchannel_rosbridge.so`
Expected: Both files should exist (assuming ROS2 env is sourced).

- [ ] **Step 2: Verify manual override**

Run: `cmake .. -DBUILD_WITH_CHANNEL_ROS2=OFF && make -j$(nproc)`
Verify: `lib/libchannel_ros2.so` is NOT updated or is removed, while `rosbridge` remains.
