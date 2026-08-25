# Cartesian ↔ Frenet / SL Basics

## Purpose

Convert global Cartesian coordinates $(x,y)$ into road-relative coordinates $(s,l)$, and back.

```text
Cartesian: where am I in the world?
Frenet:    where am I relative to the road?
```

## Meaning

```text
s = distance along the reference line
l = signed lateral offset from the reference line
```

Positive $l$ is to the **left** of the reference direction.

---

# XY → SL

## 1. Why Projection?

For a Cartesian point $P$, first find the closest point $Q$ on the reference line.

Then:

```text
position of Q along road → s
P relative to Q sideways → l
```

## 2. Project onto One Segment

Segment:

$$
A\rightarrow B
$$

Direction:

$$
d=B-A
$$

Relative vector:

$$
r=P-A
$$

Projection ratio:

$$
t=\frac{r\cdot d}{\lVert d\rVert^2}
$$

Why dot product?

> $r\cdot d$ extracts the component of $r$ along the segment direction.

Clamp:

$$
0\le t\le1
$$

Closest point on the finite segment:

$$
Q=A+t(B-A)
$$

## 3. Find the Closest Segment

Repeat projection for every segment and choose the $Q$ with minimum:

$$
\lVert P-Q\rVert^2
$$

## 4. Compute s

If $Q$ lies at ratio $t$ on segment $(i,i+1)$:

$$
s=s_i+t(s_{i+1}-s_i)
$$

## 5. Compute Signed l

Segment yaw:

$$
\psi=\mathrm{atan2}(\Delta y,\Delta x)
$$

Left unit normal:

$$
n=
\begin{bmatrix}
-\sin\psi\\
\cos\psi
\end{bmatrix}
$$

Then:

$$
l=(P-Q)\cdot n
$$

Why dot product again?

> This time it extracts the component in the **lateral normal direction**.

So:

```text
P(x, y)
  ↓ closest projection Q
Q position along road → s
(P - Q) along normal  → l
```

---

# SL → XY

## 1. Find Reference Position at s

Locate the segment containing $s$, then linearly interpolate the reference position:

$$
P_r(s)=[x_r(s),y_r(s)]^T
$$

## 2. Move l Along the Normal

$$
\boxed{P=P_r+l\,n_r}
$$

or

$$
x=x_r-l\sin\psi_r
$$

$$
y=y_r+l\cos\psi_r
$$

So:

```text
(s, l)
  ↓
find reference point at s
  ↓
move l meters along left normal
  ↓
(x, y)
```

---

# Whole Memory Chain

```text
XY → SL:
point P
  ↓ project onto reference line
closest Q
  ↓
Q along road → s
P-Q along normal → l

SL → XY:
s → reference point Pr
l → move along normal nr
P = Pr + l nr
```

## Output

A coordinate bridge:

$$
(x,y)\leftrightarrow(s,l)
$$

## Why Next Step Is Needed

The current conversion is based on straight polyline segments.

For Sampling Planner we need smooth and consistent road geometry, so Chapter 10 first upgrades the Reference Line to a cubic spline, then Chapter 11 upgrades XY ↔ SL using that same spline.
