# Reference Line Basics

## Purpose

Turn discrete road-center points into an ordered geometric reference that can be queried by longitudinal distance $s$.

## Input

```text
[(x0, y0), (x1, y1), ...]
```

## 1. Cumulative s

For adjacent points:

$$
\Delta s_i=\sqrt{(x_i-x_{i-1})^2+(y_i-y_{i-1})^2}
$$

$$
s_i=s_{i-1}+\Delta s_i
$$

So every reference point gets a longitudinal coordinate.

## 2. Yaw

Current baseline uses the forward segment direction:

$$
\psi_i=\mathrm{atan2}(y_{i+1}-y_i,\ x_{i+1}-x_i)
$$

## 3. Curvature

Geometric meaning:

$$
\kappa=\frac{d\psi}{ds}
$$

Current discrete baseline uses a centered difference:

$$
\kappa_i\approx\frac{\psi_{i+1}-\psi_{i-1}}{s_{i+1}-s_{i-1}}
$$

with angle normalization.

## ReferencePoint

```text
ReferencePoint
= (x, y, s, yaw, curvature)
```

## Query by s

Given an $s$ between two stored points, the current baseline linearly interpolates:

```text
x, y, yaw, curvature
```

## Output

```text
discrete XY points
      ↓
ReferencePoint[]
(x, y, s, yaw, curvature)
```

## Why Next Step Is Needed

Cartesian $(x,y)$ is inconvenient for road-relative planning.

We want to describe a point as:

```text
how far along the reference line? → s
how far sideways from it?        → l
```

That leads to Frenet / SL coordinates.

## Baseline Limitation

The current reference line is a piecewise-linear polyline with discrete yaw/curvature estimates.

Chapter 10 upgrades this to a smooth spline; this note intentionally keeps the old implemented baseline.
