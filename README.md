# MASSIVE DYNAMICS

## Important Note

This was a project I started in order to practice modern C\++ (namely C\++23 with some C\++26 proposals like `constexpr for`), but I got too disappointed along the way with where C\++ is heading and the current developer experience. Most notably modules are a huge pain in the rear, editor support is terribly bad and you still have to care about dependencies since it doesn't support circular references for example. Along the way I gave un on CMake support for modules, then on modules themselves, then on C\++20 in itself. It still feels like the 80's.

This project is currently a tech demo, but I decided to move forward with a different technology than C\++. I'm looking forward to see what comes out of CppFront, am very excited about Rust, and also I am quite happy where the C# ecosystem is heading now with NativeAOT having more attention and compiling on the major gaming consoles (so we can move away from BRUTE finally). The status of this project is *abandoned* and there might be some value in the SIMD routines in it. I might start a C# port of it at some point.

# OLD README:

**C\++20-compilable, C\++26-optimized, 256-bit (AVX), multi-threaded and cache-optimized physics simulator. THE FASTEST EVER<sub>●</sub>**

## Visual debugger

Massive Dynamics ships with a SDL2-powered visual debugger:

![Visual Debugger Image](visual_debugger.png)

The visual debugger is not particularly optimized. It should use hardware acceleration for rendering when available however.

## Unit tests

Massive Dynamics is tested using Catch.

**Current test coverage:**

```
Lines:    __________________________________________________ 0%
Routines: __________________________________________________ 0%
Classes:  __________________________________________________ 0%
Files:    __________________________________________________ 0%
█
```

## Compiling with a C\++20-compliant compiler

The project should be compilable without any configuration (beside CMaking) or preprocessor definition by any C\++20-compliant compiler. A decent C\++20 optimizing compiler should produce code that in practice is as fast as a C\++26-compliant compiler. Optimization flags will naturally affect this.

## C\++26 Standard feature usage

- std::simd (TODO)
- ranges (TODO)

Massive Dynamics is coded to take advantage of language additions that (hopefully will be/) are defined in the C\++26 standard. Most notably it expects the compiler to statically unroll loops in `constexpr`s for guaranteed maximum performance. While C\++26 provides explicit ways to define those loops, in C\++20 it's not possible to declare them as `constexpr` directly (it's possible with extra metaprogramming though), so the C\++26 features are guarded behind preprocessor blocks and when compiling with a C\++20 compiler we rely solely on the compiler to optimize those. Note however that a decent compiler will optimize most (if not all of these expressions) anyway.

## C\++20 vs. C\++26 performance difference

While some of the code was written with guarantees in mind that (hopefully will be/) are available in C\++26, you can compile this project in any C\++20-compliant compiler and provided optimizations are enabled most modern compilers will emit equivalently performant code. Please note that some compilers might compile SIMD intrinsics very differently, especially "sequence" instructions (which are avoided to a reasonable maximum).