# Sampling Planner

## Purpose

Generate multiple smooth lateral candidate paths from the current ego state, then convert them back to Cartesian paths for later constraint checking and cost selection.

```text
VehicleState
  ↓
initial Frenet state (s, l, l', l'')
  ↓
sample target l + target length
  ↓
quintic polynomial
  ↓
Frenet Candidates
  ↓
Cartesian Paths
```

---

# 1. Ego Cartesian State → Initial Frenet State

Position in Frenet form:

$$
P(s)=P_r(s)+l(s)n_r(s)
$$

Differentiate with respect to **reference-line** $s$:

$$
P'=(1-\kappa_r l)t_r+l'n_r
$$

So $P'$ has:

```text
reference tangent component = 1 - κr l
reference normal component  = l'
```

Let the heading difference be:

$$
\Delta\psi=\psi-\psi_r
$$

Then:

$$
\tan\Delta\psi=
\frac{l'}{1-\kappa_r l}
$$

Therefore:

$$
\boxed{
l'=(1-\kappa_r l)\tan\Delta\psi
}
$$

Memory:

> $l'$ mainly tells how lateral offset changes because ego heading differs from the reference heading.

---

# 2. Where Does $l''$ Come From?

Define:

$$
A=1-\kappa_r l
$$

Then:

$$
l'=A\tan\Delta\psi
$$

Differentiate again with respect to reference $s$:

$$
l''
=
A'\tan\Delta\psi
+
A\sec^2\Delta\psi
\frac{d\Delta\psi}{ds}
$$

with:

$$
A'=-(\kappa_r'l+\kappa_r l')
$$

Because:

$$
\Delta\psi=\psi-\psi_r
$$

we have:

$$
\frac{d\Delta\psi}{ds}
=
\frac{d\psi}{ds}-\kappa_r
$$

Ego curvature is defined on the ego path itself:

$$
\boxed{
\kappa=\frac{d\psi}{ds_{ego}}
}
$$

and the small ego-path distance projects onto the reference tangent as:

$$
ds_{ego}\cos\Delta\psi=A\,ds
$$

so:

$$
\boxed{
\frac{ds_{ego}}{ds}
=
\frac{A}{\cos\Delta\psi}
}
$$

Therefore:

$$
\frac{d\Delta\psi}{ds}
=
\kappa\frac{A}{\cos\Delta\psi}
-
\kappa_r
$$

and finally:

$$
\boxed{
l''
=
-(\kappa_r'l+\kappa_r l')\tan\Delta\psi
+
\frac{A}{\cos^2\Delta\psi}
\left(
\kappa\frac{A}{\cos\Delta\psi}
-
\kappa_r
\right)
}
$$

where:

$$
A=1-\kappa_r l
$$

Memory:

```text
l'  → current relative heading
l'' → how that relative geometry keeps changing
      → ego curvature κ
      → reference curvature κr
      → reference curvature change κr'
```

Angles in these formulas use **radians**. Curvature has unit $1/m$.

Sanity check: if ego lies exactly on the reference line with the same heading and curvature,

$$
l=0,\quad \Delta\psi=0,\quad \kappa=\kappa_r
$$

then:

$$
l'=0,\qquad l''=0
$$

---

# 3. Lateral Sampling

Current baseline samples:

$$
l_f\in\{-1.5,-0.75,0,0.75,1.5\}\;m
$$

and reference-line horizons:

$$
S\in\{10,20\}\;m
$$

So:

$$
5\times2=10
$$

candidate paths are generated.

Longer $S$ means the same lateral move can be completed more gradually.

---

# 4. Quintic Connection

Use:

$$
l(\sigma)=
a_0+a_1\sigma+a_2\sigma^2+a_3\sigma^3+a_4\sigma^4+a_5\sigma^5
$$

Six coefficients satisfy six boundary conditions:

```text
start:  l0, l0', l0''
target: lf, lf', lf''
```

Current target derivatives are:

$$
l_f'=0,\qquad l_f''=0
$$

so each candidate finishes parallel to the reference line without lateral curvature-state change in Frenet coordinates.

Sampling resolution:

$$
\Delta s=0.5\;m
$$

---

# 5. Whole Memory Chain

```text
ego x, y
  ↓ projection
s, l

ego yaw - reference yaw
  ↓
Δψ
  ↓
l'

ego curvature + reference geometry
  ↓
l''

(s, l, l', l'')
  ↓
sample target l and length
  ↓
quintic polynomial
  ↓
10 Frenet Candidates
  ↓
Cartesian Paths
```

## Output

A set of smooth geometric candidate paths ready for:

```text
Hard Constraints
  ↓
Soft Cost
  ↓
Best Path
```
