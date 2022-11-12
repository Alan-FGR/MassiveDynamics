# MASSIVE DYNAMICS

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
Files:    __________________________________________________ 0% █
```

## Compiling with a C\++20-compliant compiler

The project should be compilable without any configuration (beside CMaking) or preprocessor definition by any C\++20-compliant compiler. A decent C\++20 optimizing compiler should produce code that in practice is as fast as a C\++26-compliant compiler. Optimization flags will naturally affect this.

## C\++26 Standard feature usage

- std::simd (TODO)
- ranges (TODO)

Massive Dynamics is coded to take advantage of language additions that (hopefully will be) are defined in the C\++26 standard. Most notably it expects the compiler to statically unroll loops in `constexpr`s for guaranteed maximum performance. While C\++26 provides explicit ways to define those loops, in C\++20 it's not possible to declare them as `constexpr` directly (it's possible with extra metaprogramming though), so the C\++26 features are guarded behind preprocessor blocks and when compiling with a C\++20 compiler we rely solely on the compiler to optimize those. Note however that a decent compiler will optimize most (if not all of these expressions) anyway.

## C\++20 vs. C\++26 performance difference

While some of the code was written with guarantees in mind that (hopefully will be) are available in C\++26, you can compile this project in any C\++20-compliant compiler and provided optimizations are enabled most modern compilers will emit equivalently performant code. Please note that some compilers might compile SIMD intrinsics very differently, especially "sequence" instructions (which are avoided to a reasonable maximum).

# Using Massive Dynamics

## Creating a simulation

## Instantiating bodies and shapes

## Updating bodies and shapes

## Ticking the simulation

## Retrieving the data

## Receiving callbacks

## Implementation details

### False sharing avoidance

False sharing is avoided by distributing thread operation segments distantly so they are always reading from different cached lines. Alternatively or additionally read/write swap buffers are used for maximum guarantee of avoidance at the cost of memory usage.

## Thanks and credits

For this project in particular I would like to thank especially Erin Catto for saying that programming a physics engine takes "finesse". That gave me motivation to happily continue when everything was looking so unnatural and disappointing.

I am also grateful for everybody who was part of my journey as a software engineer; for the people who publish awesome code under open source licenses so we can learn from them (and steal a snippet or two :wink:); and for the people who publish technical articles or somehow share their knowledge with others. You know who you are!