# Design Spec: Variadic SUBSCRIBE Macro Fix

**Goal:** Resolve the compilation error caused by passing an optional `ExecutionPolicy` argument to the `SUBSCRIBE` macro, which currently only accepts two parameters.

## Problem Statement
The `MessageBus::Subscribe` method was recently updated to accept an optional `Framework::ExecutionPolicy`. However, the `SUBSCRIBE` macro in `src/core/framework/framework.h` is still defined with exactly two parameters: `topic` and `callback`. This prevents developers from using the macro to specify background execution policies, leading to compilation failures in files like `src/mainwindow/display/display_cost_map.cpp`.

## Proposed Solution
Redefine the `SUBSCRIBE` macro as a variadic macro using `__VA_ARGS__`. This allows the macro to transparently pass any number of additional arguments (specifically the optional `ExecutionPolicy`) to the underlying `MessageBus::Subscribe` call.

### Architecture & Components
- **File:** `src/core/framework/framework.h`
- **Mechanism:** Variadic Macros (`__VA_ARGS__`) with comma deletion (`##__VA_ARGS__`).

### Data Flow / Logic
1.  Developer calls `SUBSCRIBE(topic, callback)` or `SUBSCRIBE(topic, callback, policy)`.
2.  The macro expands, capturing all arguments after `callback` into `__VA_ARGS__`.
3.  The expansion calls `GetMessageBusInstance()->Subscribe<ArgType>(...)`, appending `__VA_ARGS__` at the end.
4.  If `__VA_ARGS__` is empty, the `##` extension removes the preceding comma to maintain syntax validity.

## Design Details
```cpp
#define SUBSCRIBE(topic, callback, ...) \
  [&]() { \
    auto&& cb = callback; \
    using CallbackType = std::decay_t<decltype(cb)>; \
    using ArgType = typename Framework::detail::lambda_traits<CallbackType>::arg_type; \
    return GetMessageBusInstance()->Subscribe<ArgType>(topic, \
        std::function<void(const ArgType&)>(std::forward<decltype(cb)>(cb)), ##__VA_ARGS__); \
  }()
```

## Testing Plan
1.  **Static Analysis:** Verify that files using the 3-argument `SUBSCRIBE` (e.g., `display_cost_map.cpp`) now compile successfully.
2.  **Unit Tests:** Run `framework_test` to ensure existing 2-argument subscriptions still work correctly and the new background policy test passes.
3.  **Manual Verification:** Check that the background thread pool is correctly utilized when `kBackground` is passed via the macro.

## Success Criteria
- The project compiles without macro argument mismatch errors.
- Both 2-argument and 3-argument usage of `SUBSCRIBE` works as expected.
- No regression in existing message bus functionality.
