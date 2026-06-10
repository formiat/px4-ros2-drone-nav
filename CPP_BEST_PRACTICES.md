# C++ Best Practices v2 (Modern C++17/20/23)

A standalone guide for writing maintainable, safe, testable, and performant C++.

Default assumption: use C++20 as the practical baseline, adopt C++23 features when the project toolchain supports them, and keep C++17 compatibility only when a dependency or deployment target requires it.

---

## Table of Contents

1. [Engineering Priorities](#engineering-priorities)
2. [Toolchain, Build, and CI](#toolchain-build-and-ci)
3. [Project Structure and Dependencies](#project-structure-and-dependencies)
4. [Headers, Translation Units, and Modules](#headers-translation-units-and-modules)
5. [Ownership and Lifetimes](#ownership-and-lifetimes)
6. [Resource Management and RAII](#resource-management-and-raii)
7. [Initialization and Object State](#initialization-and-object-state)
8. [Types, Constants, and Conversions](#types-constants-and-conversions)
9. [Function and API Design](#function-and-api-design)
10. [Classes and Object-Oriented Design](#classes-and-object-oriented-design)
11. [Templates, Concepts, and Generic Code](#templates-concepts-and-generic-code)
12. [Error Handling](#error-handling)
13. [Containers, Algorithms, and Ranges](#containers-algorithms-and-ranges)
14. [Strings, Views, and Formatting](#strings-views-and-formatting)
15. [Numeric Safety](#numeric-safety)
16. [Concurrency and Async Code](#concurrency-and-async-code)
17. [Real-Time, Embedded, and Robotics Constraints](#real-time-embedded-and-robotics-constraints)
18. [Performance](#performance)
19. [Security](#security)
20. [Preprocessor, Macros, and Conditional Compilation](#preprocessor-macros-and-conditional-compilation)
21. [Casting and Low-Level Code](#casting-and-low-level-code)
22. [Testing, Static Analysis, and Quality Gates](#testing-static-analysis-and-quality-gates)
23. [Documentation and Maintainability](#documentation-and-maintainability)
24. [Quick Reference](#quick-reference)

---

## Engineering Priorities

Prefer this order unless the project has stronger domain-specific constraints:

1. Correctness
2. Simplicity
3. Explicit ownership and lifetime semantics
4. Testability
5. Observability and diagnosability
6. Performance based on measurement
7. Minimal cleverness

### Do

- Optimize for code that a future maintainer can reason about quickly.
- Make invalid states unrepresentable where practical.
- Prefer standard library facilities over custom infrastructure.
- Keep implementation details private and APIs small.
- Write code that works under sanitizers and high warning levels.
- Treat undefined behavior as a correctness bug, not a performance trick.

### Don't

- Do not rely on "works on my compiler" behavior.
- Do not hide ownership or lifetime assumptions in comments only.
- Do not trade safety for performance without a benchmark and a documented reason.
- Do not create abstractions before there are at least two real uses or a clear domain boundary.

---

## Toolchain, Build, and CI

Use the build system to enforce quality. A style guide that is not checked automatically will drift.

### Do

- Build with at least two compilers when possible, usually Clang and GCC on Linux.
- Enable warnings and treat project warnings as errors in CI:

  ```cmake
  target_compile_options(my_target PRIVATE
      -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion
      -Wshadow -Wnon-virtual-dtor -Wold-style-cast)
  ```

- Use modern CMake target-based configuration:

  ```cmake
  add_library(robot_control src/controller.cpp)
  target_compile_features(robot_control PUBLIC cxx_std_20)
  target_include_directories(robot_control PUBLIC include)
  target_link_libraries(robot_control PUBLIC Eigen3::Eigen)
  ```

- Keep compiler flags, include paths, definitions, and dependencies attached to targets.
- Generate `compile_commands.json` for tooling.
- Run formatter and static analysis in CI.
- Use reproducible dependency management: system packages, `FetchContent`, package managers, or vendored dependencies with a clear policy.
- Pin toolchain versions for production builds.

### Don't

- Do not use global CMake state such as `include_directories()` or `add_definitions()` for new code.
- Do not make CI use a different build path than developers.
- Do not disable warnings globally for third-party code and project code together.
- Do not depend on transitive includes or transitive link libraries.
- Do not accept "warning-only" builds for production branches.

---

## Project Structure and Dependencies

Structure should make ownership of code, tests, generated files, and public APIs obvious.

### Do

- Separate public headers from private implementation details:

  ```text
  include/my_project/controller.hpp
  src/controller.cpp
  src/controller_internal.hpp
  tests/controller_test.cpp
  ```

- Keep public API headers minimal and stable.
- Put generated code in a clearly named generated directory.
- Wrap third-party APIs behind project-owned interfaces when the dependency is volatile or invasive.
- Prefer dependency injection for hardware, network, clock, and file-system boundaries.
- Keep one clear namespace root per project:

  ```cpp
  namespace autopilot::control {
  class Controller;
  }
  ```

### Don't

- Do not expose third-party types in public APIs unless you intentionally commit to that dependency.
- Do not mix tests, generated files, and production sources in the same directory without convention.
- Do not put large implementations in public headers unless templates or performance constraints require it.
- Do not use `using namespace` in headers.

---

## Headers, Translation Units, and Modules

Headers are still the practical default in many production C++ projects. Modules can be valuable, but tooling and ecosystem support vary.

### Do

- Use `#pragma once` or include guards consistently.
- Include what you use.
- Forward-declare in headers when it reduces coupling and does not obscure the API.
- Keep headers self-contained: each public header should compile when included alone.
- Prefer modules only when the compiler, build system, IDE, package manager, and deployment workflow support them reliably.
- Use internal linkage for translation-unit-local helpers:

  ```cpp
  namespace {
  constexpr int kRetryLimit{3};
  }
  ```

### Don't

- Do not rely on transitive includes.
- Do not put non-inline non-template function definitions in headers.
- Do not put `using namespace std;` in any header.
- Do not adopt C++20 modules in a production project just because they are modern. Validate build times, tooling, packaging, and CI first.

---

## Ownership and Lifetimes

Most serious C++ bugs come from unclear ownership, dangling references, or invalid assumptions about object lifetime.

### Do

- Prefer automatic storage duration for local objects.
- Use `std::unique_ptr<T>` for exclusive heap ownership.
- Use `std::shared_ptr<T>` only when shared ownership is real.
- Use `std::weak_ptr<T>` to break ownership cycles.
- Use raw pointers and references as non-owning observers only.
- Prefer references for required non-null parameters.
- Use pointers for nullable non-owning parameters.
- Use `std::span<T>` for non-owning contiguous ranges.
- Use `std::string_view` for non-owning read-only string parameters.
- Document any API that stores a view, reference, pointer, iterator, or callback.

  ```cpp
  class Parser {
  public:
      // The caller must ensure buffer outlives Parser.
      explicit Parser(std::span<const std::byte> buffer) : buffer_{buffer} {}

  private:
      std::span<const std::byte> buffer_;
  };
  ```

### Don't

- Do not use raw `new` or `delete` in application code.
- Do not pass `std::shared_ptr<T>` just to avoid thinking about lifetime.
- Do not store `std::string_view` or `std::span` unless the owner lifetime is explicit and tested.
- Do not return pointers, references, views, or iterators to local objects.
- Do not create ownership cycles with `shared_ptr`.
- Do not use `malloc`, `calloc`, `realloc`, or `free` for C++ objects.

### Preferred ownership vocabulary

| Meaning | Preferred form |
|---|---|
| Required object, no ownership transfer | `T&` or `const T&` |
| Optional object, no ownership transfer | `T*` or `const T*` |
| Exclusive ownership transfer | `std::unique_ptr<T>` |
| Shared ownership transfer/share | `std::shared_ptr<T>` |
| Non-owning contiguous data | `std::span<T>` |
| Non-owning read-only string | `std::string_view` |
| Optional value | `std::optional<T>` |

---

## Resource Management and RAII

Every resource should have an owner object that releases it in its destructor.

### Do

- Use RAII for files, sockets, locks, memory, subscriptions, handles, and hardware resources.
- Prefer the Rule of Zero. If a class does not directly manage a resource, let the compiler generate special member functions.
- If a class manages a resource directly, define or delete all relevant special members.
- Use `std::lock_guard`, `std::unique_lock`, or `std::scoped_lock` for mutexes.
- Make destructors `noexcept`.
- Use custom deleters for C resources:

  ```cpp
  using FilePtr = std::unique_ptr<FILE, decltype(&std::fclose)>;

  FilePtr open_file(const char* path) {
      return FilePtr{std::fopen(path, "rb"), &std::fclose};
  }
  ```

### Don't

- Do not manually pair acquire/release across distant code paths.
- Do not call `lock()` and `unlock()` manually in normal code.
- Do not let destructors throw.
- Do not use `setjmp` or `longjmp` in C++ code that owns resources.
- Do not hide resource ownership behind ambiguous names like `Handle` unless semantics are documented.

---

## Initialization and Object State

Objects should be initialized once, completely, and as close as possible to declaration.

### Do

- Initialize variables at declaration.
- Use brace initialization to prevent narrowing:

  ```cpp
  int retries{3};
  std::vector<int> ids{1, 2, 3};
  ```

- Use default member initializers:

  ```cpp
  struct ControlConfig {
      double max_velocity_mps{2.0};
      double max_accel_mps2{1.0};
      bool enable_limits{true};
  };
  ```

- Use member initializer lists in constructors.
- Prefer factory functions when construction can fail:

  ```cpp
  class Device {
  public:
      static std::expected<Device, DeviceError> open(std::string_view path);

  private:
      explicit Device(NativeHandle handle) : handle_{handle} {}
      NativeHandle handle_;
  };
  ```

### Don't

- Do not leave fundamental types uninitialized.
- Do not use `memset` to initialize non-trivial C++ objects.
- Do not put complex initialization logic in constructors if it can fail in multiple ways.
- Do not create half-valid objects that require a separate `init()` call unless a framework forces it.

---

## Types, Constants, and Conversions

Use the type system to encode intent.

### Do

- Use `nullptr`, not `NULL` or `0`.
- Use `enum class` for enumerations.
- Use `using`, not `typedef`.
- Use `constexpr` for compile-time constants.
- Use `const` by default for values that should not change.
- Use strong domain types for values with units:

  ```cpp
  struct Meters {
      double value{};
  };

  struct Seconds {
      double value{};
  };
  ```

- Use `std::chrono` for durations and timestamps.
- Use fixed-width integer types for serialization and protocols:

  ```cpp
  std::uint32_t sequence_number{};
  ```

- Mark single-argument constructors `explicit`.
- Use `[[nodiscard]]` for important return values.

### Don't

- Do not use C-style casts.
- Do not rely on implicit narrowing conversions.
- Do not use plain `int` or `long` for binary formats, wire protocols, or persistent storage.
- Do not use `volatile` for thread synchronization.
- Do not use `bool` parameters when the call site becomes unclear:

  ```cpp
  // Bad
  start_motor(true);

  // Better
  start_motor(StartMode::Calibrated);
  ```

---

## Function and API Design

Good APIs make correct use easy and incorrect use hard.

### Do

- Keep functions small and focused.
- Prefer returning values over output parameters.
- Pass small cheap types by value.
- Pass large read-only objects by `const&`.
- Pass strings as `std::string_view` when the function does not store them.
- Pass contiguous buffers as `std::span`.
- Use `std::optional<T>` for optional results.
- Use `std::expected<T, E>` for recoverable failures when C++23 is available.
- Mark non-mutating member functions `const`.
- Mark functions `noexcept` only when they truly cannot throw and can maintain that guarantee.
- Use named option structs when a function has multiple parameters of the same type:

  ```cpp
  struct PlannerOptions {
      double max_speed_mps{};
      double max_accel_mps2{};
      bool avoid_dynamic_obstacles{true};
  };

  Plan make_plan(const Map& map, const PlannerOptions& options);
  ```

### Don't

- Do not use raw output pointers when a return value is clearer.
- Do not expose nullable pointer parameters without documenting null behavior.
- Do not accept `std::shared_ptr<T>` unless the function shares or stores ownership.
- Do not return `const T` by value.
- Do not overload functions in ways that make calls ambiguous.
- Do not use default arguments to hide materially different behavior.

---

## Classes and Object-Oriented Design

Prefer simple value types and composition. Use inheritance for substitutability, not code reuse.

### Do

- Prefer small cohesive classes.
- Prefer composition over inheritance.
- Use abstract base classes for stable interfaces:

  ```cpp
  class Clock {
  public:
      virtual ~Clock() = default;
      virtual std::chrono::steady_clock::time_point now() const = 0;
  };
  ```

- Use `override` on all overriding functions.
- Use `final` when a class or override should not be extended.
- Make base class destructors public and virtual, or protected and non-virtual.
- Keep invariants local to the class and enforce them at construction.
- Use aggregates for passive data.

### Don't

- Do not derive from concrete classes just to reuse implementation.
- Do not create deep inheritance trees.
- Do not use protected data members.
- Do not expose mutable public data except in simple aggregate types.
- Do not make everything virtual by default.
- Do not put heavy business logic in constructors or destructors.

---

## Templates, Concepts, and Generic Code

Generic code should be constrained, readable, and justified.

### Do

- Use concepts to express requirements:

  ```cpp
  template <std::floating_point T>
  T clamp_unit(T value) {
      return std::clamp(value, T{0}, T{1});
  }
  ```

- Use `if constexpr` for compile-time branching.
- Use generic lambdas where they improve locality.
- Put template definitions in headers or modules.
- Prefer simple function templates over complex class template machinery.
- Use `static_assert` with actionable messages.

### Don't

- Do not write unconstrained templates when concepts can state intent.
- Do not over-genericize code with one concrete use.
- Do not specialize standard library templates unless the standard explicitly allows it.
- Do not use template metaprogramming when a normal function or type is enough.
- Do not hide slow compile-time machinery behind innocent-looking includes.

---

## Error Handling

Choose the error mechanism based on whether the caller can reasonably recover and what the project runtime permits.

### Do

- Use exceptions for exceptional failures in codebases where exceptions are allowed and the caller may recover at a higher layer.
- Use `std::expected<T, E>` for expected recoverable failures in APIs where explicit handling is preferred.
- Use `std::optional<T>` only when absence is the only failure reason.
- Use assertions for programmer errors and broken invariants.
- Use `std::error_code` when interoperating with standard or OS APIs that already use it.
- Throw by value and catch by `const&`:

  ```cpp
  try {
      load_config(path);
  } catch (const ConfigError& error) {
      report(error);
  }
  ```

- Include context in errors. "open failed" is weak; "failed to open /dev/ttyUSB0: permission denied" is useful.
- Define project-level error categories instead of leaking low-level errors everywhere.

### Don't

- Do not use exceptions for normal control flow.
- Do not swallow exceptions silently.
- Do not use `catch (...)` unless you immediately translate, log and rethrow, or terminate intentionally.
- Do not throw from destructors.
- Do not use `errno` as a general C++ error mechanism.
- Do not return magic values such as `-1` or `nullptr` without a documented reason.

### Exceptions disabled

Some embedded, safety-critical, or real-time codebases disable exceptions. In that case:

- Use `std::expected`, project-specific result types, or error callbacks.
- Keep failure paths explicit.
- Enforce return-value handling with `[[nodiscard]]`.
- Avoid APIs that can fail implicitly during hot paths.

---

## Containers, Algorithms, and Ranges

Use containers and algorithms that match data access patterns.

### Do

- Use `std::vector<T>` as the default sequence container.
- Use `std::array<T, N>` for fixed-size arrays.
- Use `std::deque<T>` when stable references and efficient front insertion are needed.
- Use `std::unordered_map` for average-case lookup by key and `std::map` when ordering is required.
- Use `std::span<T>` for non-owning contiguous data.
- Use standard algorithms:

  ```cpp
  const auto found = std::ranges::find_if(items, [](const Item& item) {
      return item.ready();
  });
  ```

- Reserve capacity when the final size is known or can be estimated.
- Use erase-remove or C++20 `std::erase_if`:

  ```cpp
  std::erase_if(values, [](int value) { return value < 0; });
  ```

### Don't

- Do not use `std::list` by default. It is usually slower due to poor cache locality.
- Do not use C arrays in new code when `std::array`, `std::vector`, or `std::span` fits.
- Do not invalidate iterators while using them.
- Do not store references in standard containers. Use values, pointers, or `std::reference_wrapper` intentionally.
- Do not use `std::vector<bool>` unless you specifically want its packed proxy behavior.

---

## Strings, Views, and Formatting

String ownership and lifetime must be explicit.

### Do

- Use `std::string` for owned strings.
- Use `std::string_view` for read-only parameters that are not stored.
- Use `std::format` or the `{fmt}` library for formatting when supported.
- Use raw string literals for regexes and paths:

  ```cpp
  const std::regex date_pattern{R"(\d{4}-\d{2}-\d{2})"};
  ```

- Convert to null-terminated strings only at C API boundaries.

### Don't

- Do not return `std::string_view` to a temporary string.
- Do not store `std::string_view` from a caller unless the lifetime contract is explicit.
- Do not use `sprintf`, `strcpy`, `strcat`, or unchecked C string APIs.
- Do not assume `std::string_view` is null-terminated.
- Do not use `std::string::c_str()` after mutating the string or after the string is destroyed.

---

## Numeric Safety

Numeric bugs are common in robotics, networking, binary protocols, and geometry code.

### Do

- Use domain-specific types for units when mistakes are expensive.
- Use `std::chrono` duration types instead of raw time numbers.
- Use fixed-width integers for serialization.
- Validate external numeric input before conversion.
- Be explicit about signed/unsigned boundaries.
- Use checked arithmetic helpers for untrusted sizes, offsets, and allocations.
- Prefer `std::size_t` for container sizes, but avoid mixing it casually with signed indices.

### Don't

- Do not rely on signed integer overflow. It is undefined behavior.
- Do not compare signed and unsigned values casually.
- Do not cast large sizes to smaller integer types without range checks.
- Do not use floating-point equality for measured physical values:

  ```cpp
  bool nearly_equal(double a, double b, double eps) {
      return std::abs(a - b) <= eps;
  }
  ```

- Do not use `int` as a universal numeric type.

---

## Concurrency and Async Code

Data races are undefined behavior. Make ownership and synchronization obvious.

### Do

- Prefer immutable data, message passing, queues, futures, or actors over shared mutable state.
- Use `std::jthread` and `std::stop_token` for cancellable threads in C++20.
- Use RAII locks.
- Use `std::scoped_lock` for locking multiple mutexes.
- Use `std::atomic<T>` only when you understand the required memory ordering.
- Keep critical sections small.
- Define lock ordering when multiple locks exist.
- Use thread-safe queues or executors for cross-thread communication.
- Test with ThreadSanitizer where supported.

### Don't

- Do not detach threads unless process lifetime intentionally owns the work.
- Do not access shared mutable data without synchronization.
- Do not use `volatile` for synchronization.
- Do not hold locks while calling unknown user callbacks.
- Do not hold locks while doing blocking I/O unless explicitly required.
- Do not assume `std::async` always creates a new thread. Its launch policy matters.
- Do not block destructors unexpectedly in hot paths.

---

## Real-Time, Embedded, and Robotics Constraints

For robotics, simulation, control loops, and hardware integration, general C++ advice needs stricter runtime discipline.

### Do

- Separate hard real-time control loops from non-real-time application logic.
- Allocate memory before entering deterministic control loops.
- Avoid logging, heap allocation, blocking I/O, and locks in hard real-time paths.
- Use bounded queues and explicit backpressure.
- Prefer fixed-capacity containers in real-time sections when available in the project.
- Use monotonic clocks for durations and deadlines.
- Make units explicit for positions, velocities, accelerations, timestamps, and frames.
- Treat coordinate frames and conventions as part of the type/API contract.
- Validate all data received from sensors, networks, middleware, and simulation.
- Keep simulation-specific assumptions out of production hardware code.

### Don't

- Do not call dynamic allocation, filesystem, network, or logging APIs from a hard real-time callback unless proven bounded and allowed by the platform.
- Do not put blocking waits in executor callbacks without understanding the executor threading model.
- Do not assume simulation timing, sensor rates, or message ordering match hardware.
- Do not mix coordinate frames with raw `double` arrays and no type/context.
- Do not hide safety-critical state transitions behind generic callbacks.

---

## Performance

Performance work starts with measurement.

### Do

- Profile before optimizing.
- Benchmark representative workloads, not toy cases only.
- Measure debug and release builds separately.
- Prefer contiguous memory and cache-friendly access patterns.
- Use `reserve()` when growth is predictable.
- Avoid unnecessary allocations in hot paths.
- Pass views for read-only data.
- Use move semantics where ownership transfer is intended.
- Mark move constructors `noexcept` for types stored in standard containers.
- Inspect generated assembly only after profiling identifies a hotspot.

### Don't

- Do not optimize code that is not on a measured hot path.
- Do not assume hand-written loops beat standard algorithms.
- Do not assume `std::list` improves performance for insertion-heavy workloads.
- Do not use manual memory pools without measurement and tests.
- Do not add branch prediction attributes such as `[[likely]]` without evidence.
- Do not sacrifice API correctness for micro-optimizations.

---

## Security

Security-sensitive C++ should minimize raw memory manipulation and validate boundaries aggressively.

### Do

- Validate all external input.
- Use safe containers and spans instead of raw buffers.
- Use `std::filesystem::path` for path composition.
- Canonicalize and validate paths before trusting them.
- Use cryptographic libraries or OS CSPRNG APIs for security-sensitive randomness.
- Treat integer overflow in sizes and offsets as a vulnerability.
- Prefer allowlists over blocklists for commands, paths, protocols, and message types.
- Clear sensitive memory with a mechanism that the compiler cannot optimize away.

### Don't

- Do not use `system()` with user-controlled input.
- Do not use `rand()` for security.
- Do not use `std::mt19937` for cryptographic randomness.
- Do not use unsafe C string functions.
- Do not trust environment variables, file paths, network packets, or middleware messages.
- Do not keep secrets in normal `std::string` if secure erasure is required.

---

## Preprocessor, Macros, and Conditional Compilation

Use the preprocessor sparingly. It has no type safety and poor scoping.

### Do

- Use `constexpr`, `consteval`, `enum class`, templates, and inline functions instead of macros.
- Keep platform-specific code behind narrow abstraction layers.
- Use feature-detection macros in build configuration, not scattered through business logic.
- Use `static_assert` for compile-time requirements.
- Prefix unavoidable macros with a project-specific namespace-like prefix.

### Don't

- Do not use macros for constants or function-like operations.
- Do not use macros to simulate templates or reflection.
- Do not put large `#ifdef` blocks in core logic.
- Do not define short generic macro names.
- Do not let generated macros leak into public headers.

---

## Casting and Low-Level Code

Low-level code is sometimes necessary. Make it isolated, reviewed, and tested.

### Do

- Use named casts:

  ```cpp
  auto value = static_cast<double>(count);
  ```

- Use `dynamic_cast` only for intentional runtime polymorphic downcasts.
- Use `std::bit_cast` for safe bit reinterpretation of trivially copyable types.
- Isolate `reinterpret_cast` behind narrow functions with tests.
- Document every place that depends on object representation, alignment, endian, or ABI.
- Use `std::byte` for raw bytes.

### Don't

- Do not use C-style casts.
- Do not use `reinterpret_cast` for casual type punning.
- Do not use `const_cast` to modify an object that was originally const.
- Do not violate strict aliasing.
- Do not assume endian or alignment unless checked.
- Do not use pointer arithmetic when a span or iterator is suitable.

---

## Testing, Static Analysis, and Quality Gates

Testing is part of the design, not cleanup after coding.

### Do

- Write unit tests for pure logic.
- Write integration tests for middleware, filesystem, network, and hardware boundaries.
- Use regression tests for every bug fix.
- Use property-based tests for parsers, serializers, geometry, and state machines where practical.
- Use fuzzing for parsers and untrusted input.
- Run sanitizers in CI:

  ```text
  AddressSanitizer
  UndefinedBehaviorSanitizer
  ThreadSanitizer
  LeakSanitizer
  ```

- Run static analysis:

  ```text
  clang-tidy
  cppcheck
  compiler warnings
  include-what-you-use where practical
  ```

- Use coverage as a signal, not a goal by itself.
- Add benchmarks for performance-sensitive code and compare against a baseline.

### Don't

- Do not merge code that only works without sanitizers unless there is a documented tool false positive.
- Do not test only happy paths.
- Do not use sleeps as synchronization in tests when deterministic synchronization is possible.
- Do not make unit tests depend on global order or wall-clock timing.
- Do not skip tests for error paths, cancellation paths, and resource cleanup.

---

## Documentation and Maintainability

Documentation should explain contracts and decisions, not repeat syntax.

### Do

- Document ownership, lifetimes, threading, exceptions, and real-time constraints.
- Document units and coordinate frames.
- Document public API preconditions and postconditions.
- Keep comments close to the code they explain.
- Use comments to explain why, not what.
- Record important architectural decisions in short design notes.
- Keep examples buildable when possible.

### Don't

- Do not leave stale comments.
- Do not comment out dead code.
- Do not document obvious statements:

  ```cpp
  ++i; // increment i
  ```

- Do not hide important API contracts in external docs only.
- Do not let examples drift from current APIs.

---

## Quick Reference

| Area | Prefer | Avoid |
|---|---|---|
| Ownership | `unique_ptr`, values, references, RAII | raw `new`/`delete`, unclear raw owning pointers |
| Shared ownership | `shared_ptr` only when ownership is shared | passing `shared_ptr` everywhere |
| Optional data | `std::optional<T>` | sentinel values |
| Recoverable errors | `std::expected<T, E>` or exceptions by policy | ignored error codes, magic values |
| Strings | `std::string`, `std::string_view` | raw C strings in public APIs |
| Buffers | `std::span`, `std::vector`, `std::array` | raw pointer plus length unless boundary requires it |
| Constants | `constexpr`, `consteval`, `enum class` | `#define` constants |
| Formatting | `std::format` or `{fmt}` | `sprintf`, unsafe C formatting |
| Time | `std::chrono` | raw integer milliseconds everywhere |
| Concurrency | `jthread`, RAII locks, atomics with care | `detach`, `volatile`, unsynchronized shared state |
| Casting | named casts, `std::bit_cast` | C-style casts |
| Build | target-based CMake, warnings as errors | global CMake flags and transitive dependencies |
| Tests | unit, integration, fuzz, sanitizers | happy-path-only tests |
| Performance | profile, benchmark, measure | premature micro-optimization |
| Real-time | preallocation, bounded work, no blocking | heap/logging/I/O in hard real-time loops |

---

## Minimal Production Checklist

- The project builds with warnings enabled and project warnings treated as errors.
- Formatting is automated.
- Static analysis runs in CI.
- Sanitizers run in CI for supported targets.
- Public APIs document ownership, lifetime, threading, and error behavior.
- No raw owning pointers exist in application code.
- No unchecked external input reaches allocation sizes, indexes, paths, or protocol fields.
- Hot paths have benchmarks or profiling evidence.
- Real-time paths avoid allocation, blocking I/O, unbounded queues, and unexpected locks.
- Every bug fix includes a regression test unless impossible.
- Dependencies and compiler versions are pinned for production releases.

---

## Recommended References

- ISO C++ standard for the selected language version.
- C++ Core Guidelines.
- SEI CERT C++ Coding Standard.
- Compiler documentation for GCC, Clang, and MSVC warnings/sanitizers.
- Project-specific safety, real-time, and deployment requirements.
