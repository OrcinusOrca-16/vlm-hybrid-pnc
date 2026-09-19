# Cartesian ↔ Frenet / SL Basics

## Purpose

Convert global Cartesian coordinates `(x, y)` into road-relative coordinates `(s, l)`, and back.

```text
Cartesian: where am I in the world?
Frenet:    where am I relative to the reference line?
```

## Meaning

```text
s = station along the reference line
l = signed lateral offset from the reference line
```

Positive `l` is to the **left** of the reference direction.

> In the current implementation, `s` is the spline parameter built from cumulative distances between raw waypoints. It is used as an arc-length approximation.

---

# XY → SL

## 1. Find the Nearest Reference Station

For a Cartesian point:

$$
P=(x,y)^T
$$

find the reference-line station that minimizes distance to the spline:

$$
s^*=\operatorname*{argmin}_s \lVert P-P_r(s)Vert^2
$$

Then:

$$
P_r=P_r(s^*)
$$

So the longitudinal Frenet coordinate is:

$$
\boxed{s=s^*}
$$

Current `ReferenceLine.nearest_s()` does this in two stages:

```text
query point P
      ↓
find nearest raw waypoint
      ↓
build a local s interval
      ↓
ternary-search distance to the spline
      ↓
nearest s
```

The current baseline assumes the projection is locally unique.

## 2. Compute Signed Lateral Offset

Query the spline heading at `s`:

$$
\psi_r=\psi_r(s)
$$

Left unit normal:

$$
\mathbf{n}_r=
\begin{bmatrix}
-\sin\psi_r\\
\cos\psi_r
\end{bmatrix}
$$

Position error from the reference point to the Cartesian point:

$$
e=P-P_r
$$

Project this error onto the left normal:

$$
\boxed{l=e\cdot\mathbf{n}_r}
$$

Therefore:

```text
l > 0  → left of reference direction
l = 0  → on the reference line
l < 0  → right of reference direction
```

Whole XY → SL chain:

```text
P(x, y)
  ↓ nearest_s()
reference station s
  ↓ ReferenceLine.position(s), yaw(s)
Pr, nr
  ↓
l = (P - Pr) · nr
  ↓
(s, l)
```

---

# SL → XY

Given `(s, l)`, first query the smooth reference-line geometry:

$$
P_r(s)=
\begin{bmatrix}
x_r(s)\\
y_r(s)
\end{bmatrix}
$$

$$
\mathbf{n}_r(s)=
\begin{bmatrix}
-\sin\psi_r(s)\\
\cos\psi_r(s)
\end{bmatrix}
$$

Then move `l` meters along the reference-line normal:

$$
\boxed{P=P_r+l\mathbf{n}_r}
$$

Therefore:

$$
x=x_r-l\sin\psi_r
$$

$$
y=y_r+l\cos\psi_r
$$

Whole SL → XY chain:

```text
(s, l)
  ↓ query spline at s
Pr, ψr
  ↓ build left normal nr
P = Pr + l nr
  ↓
(x, y)
```

---

# Old Baseline vs Current Implementation

Old Chapter baseline:

```text
raw polyline
→ project onto every straight segment
→ choose nearest segment projection
```

Current implementation:

```text
raw waypoints
→ cubic spline ReferenceLine
→ nearest raw waypoint for coarse localization
→ local search on the spline for nearest s
→ use spline position / yaw to compute l
```

The key idea is unchanged:

$$
\boxed{
	ext{find the closest reference point first}
ightarrow
	ext{measure longitudinal station and lateral offset}
}
$$

The difference is that the current version performs the geometry on a **smooth spline** instead of treating the road as disconnected straight segments.

---

# Whole Memory Chain

```text
XY → SL:
P(x, y)
  ↓ nearest point on smooth ReferenceLine
s
  ↓ lateral projection onto nr
l

SL → XY:
s → Pr(s), ψr(s), nr(s)
l → move along nr
P = Pr + l nr
```

## Output

$$
\boxed{(x,y)left\rightarrow(s,l)}
$$

This provides the coordinate bridge used by the Sampling Planner:

```text
ego Cartesian state
      ↓
initial Frenet state
      ↓
sample Frenet Candidates
      ↓
Cartesian Candidate Paths
```
