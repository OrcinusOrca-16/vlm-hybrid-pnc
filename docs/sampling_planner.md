# Sampling Planner

## Goal

在 Frenet / SL 空间生成多条 Candidate Path：

$$
l=l(s)
$$

通过 constraint / collision / cost 选择 best path。

Candidate 最初表示为：

$$
(s,l,l',l'')
$$

Cartesian PathPoint 最终需要：

$$
(x,y,s,\psi,\kappa)
$$

因此核心目标：

$$
\boxed{
(s,l,l',l'')
\rightarrow
(x,y,s,\psi,\kappa)
}
$$

---

## Reference Line

**目的：SL 坐标系建立在 Reference Line 上，因此 Candidate 从 SL 转换到全局 XY 时，必须使用同一 $s$ 处的 Reference Line 几何量。**

所有带下标 $r$ 的量都属于 Reference Line：

$$
P_r=
\begin{bmatrix}
x_r\\
y_r
\end{bmatrix}
$$

$$
\psi_r,\quad
\kappa_r,\quad
\kappa_r'
$$

给定 $s$ 后，可由 `ReferenceLine.query(s)` 得到这些量。

单位切向量：

$$
\mathbf t_r=
\begin{bmatrix}
\cos\psi_r\\
\sin\psi_r
\end{bmatrix}
$$

单位左法向量：

$$
\mathbf n_r=
\begin{bmatrix}
-\sin\psi_r\\
\cos\psi_r
\end{bmatrix}
$$

并且：

$$
\frac{d\mathbf t_r}{ds}
=
\kappa_r\mathbf n_r
$$

$$
\frac{d\mathbf n_r}{ds}
=
-\kappa_r\mathbf t_r
$$

当前 $\kappa_r'$ 对 spline 上连续的 $\kappa_r(s)$ 做数值差分：

$$
\kappa_r'(s)
\approx
\frac{
\kappa_r(s+\Delta s)-\kappa_r(s-\Delta s)
}{2\Delta s}
$$

端点使用单边差分。

> 以下 Frenet 几何按 $s$ 为 Reference Line 弧长参数推导。当前实现以 raw waypoint 累计距离作为 spline 参数，并将其作为弧长近似。

---

## Frenet Variables

**目的：$l$ 描述横向位置；$l'$ 描述轨迹方向；$l''$ 描述方向变化，用于进一步求 yaw 和 curvature。**

$$
l=l(s)
$$

表示相对于 Reference Line 的横向位置。

$$
l'=\frac{dl}{ds}
$$

表示横向位置随 $s$ 的变化趋势，主要影响 Candidate 方向。

$$
l''=\frac{d^2l}{ds^2}
$$

表示 $l'$ 随 $s$ 的变化趋势，主要影响 Candidate 曲率。

注意：

$$
l' \neq \frac{dl}{dt}
$$

这里都是对 Reference Line 弧长 $s$ 的空间导数。

---

## Position — $P$

**目的：求 Candidate 在全局 XY 坐标系中的位置。**

$$
P=
\begin{bmatrix}
x\\
y
\end{bmatrix}
$$

由：

$$
\boxed{
P=P_r+l\mathbf n_r
}
$$

得到：

$$
\boxed{x=x_r-l\sin\psi_r}
$$

$$
\boxed{y=y_r+l\cos\psi_r}
$$

---

## Direction — $P'$

**目的：对位置求导得到 Candidate 切线方向，再由切线方向求 yaw。**

$$
P'=\frac{dP}{ds}
$$

得到：

$$
\boxed{
P'=
(1-\kappa_r l)\mathbf t_r
+
l'\mathbf n_r
}
$$

在 Reference Line 局部坐标系中：

$$
P'
\Longleftrightarrow
\begin{bmatrix}
1-\kappa_r l\\
l'
\end{bmatrix}
$$

因此：

$$
\boxed{
\psi=
\psi_r+
\mathrm{atan2}
\left(
l',
1-\kappa_r l
\right)
}
$$

其中：

$$
1-\kappa_r l
$$

表示 Candidate 位于弯道内侧 / 外侧造成的切向尺度变化。

---

## Direction Change — $P''$

**目的：$P''$ 描述 Candidate 切线如何变化，用于计算 curvature。**

$$
P''=\frac{d^2P}{ds^2}
$$

其中：

$$
\kappa_r'=\frac{d\kappa_r}{ds}
$$

得到：

$$
\boxed{
P''=
(-\kappa_r'l-2\kappa_r l')\mathbf t_r
+
[\kappa_r(1-\kappa_r l)+l'']\mathbf n_r
}
$$

---

## Curvature — $\kappa$

平面曲线通用曲率公式：

$$
\boxed{
\kappa=
\frac{P'\times P''}{\|P'\|^3}
}
$$

代入 $P'$ 和 $P''$：

$$
\boxed{
\kappa=
\frac{
(1-\kappa_r l)[\kappa_r(1-\kappa_r l)+l'']
+
l'(\kappa_r'l+2\kappa_r l')
}{
[(1-\kappa_r l)^2+(l')^2]^{3/2}
}
}
$$

曲率分母对应：

$$
\|P'\|^2=
(1-\kappa_r l)^2+(l')^2
$$

当它接近 $0$ 时 Frenet 几何退化，实际实现需要分母保护。

---

## Meaning of $P$

$$
P
\rightarrow
\text{Position}
\rightarrow
x,y
$$

$$
P'
\rightarrow
\text{Direction}
\rightarrow
\psi
$$

$$
P',P''
\rightarrow
\text{Direction Change}
\rightarrow
\kappa
$$

---

## Main Flow

```text
Frenet Candidate
s, l, l', l''
      ↓
Reference Line 提供
Pr, ψr, κr, κr'
      ↓
P = Pr + l nr
      ↓
x, y
      ↓ 对 s 求导
P'
      ↓ atan2
yaw ψ
      ↓ 再求导
P''
      ↓
curvature κ
      ↓
Cartesian PathPoint
x, y, s, ψ, κ
```

最终：

$$
\boxed{
(s,l,l',l'')
\rightarrow
(x,y,s,\psi,\kappa)
}
$$

---

## Appendix — Geometry Derivation

### A. Reference Line 切 / 法向量导数

Reference Line 的单位切向量：

$$
\mathbf t_r=
\begin{bmatrix}
\cos\psi_r\\
\sin\psi_r
\end{bmatrix}
$$

对 $s$ 求导：

$$
\frac{d\mathbf t_r}{ds}
=
\begin{bmatrix}
-\sin\psi_r\dfrac{d\psi_r}{ds}\\
\cos\psi_r\dfrac{d\psi_r}{ds}
\end{bmatrix}
$$

弧长参数下：

$$
\kappa_r=
\frac{d\psi_r}{ds}
$$

所以：

$$
\frac{d\mathbf t_r}{ds}
=
\kappa_r
\begin{bmatrix}
-\sin\psi_r\\
\cos\psi_r
\end{bmatrix}
$$

因此：

$$
\boxed{
\frac{d\mathbf t_r}{ds}
=
\kappa_r\mathbf n_r
}
$$

左法向量：

$$
\mathbf n_r=
\begin{bmatrix}
-\sin\psi_r\\
\cos\psi_r
\end{bmatrix}
$$

对 $s$ 求导：

$$
\frac{d\mathbf n_r}{ds}
=
\begin{bmatrix}
-\cos\psi_r\dfrac{d\psi_r}{ds}\\
-\sin\psi_r\dfrac{d\psi_r}{ds}
\end{bmatrix}
$$

代入 $d\psi_r/ds=\kappa_r$：

$$
\frac{d\mathbf n_r}{ds}
=
-\kappa_r
\begin{bmatrix}
\cos\psi_r\\
\sin\psi_r
\end{bmatrix}
$$

因此：

$$
\boxed{
\frac{d\mathbf n_r}{ds}
=
-\kappa_r\mathbf t_r
}
$$

### B. $P'$ 推导

Candidate 位置：

$$
P=P_r+l\mathbf n_r
$$

对 $s$ 求导：

$$
P'
=
P_r'
+
l'\mathbf n_r
+
l\mathbf n_r'
$$

因为 $s$ 是 Reference Line 弧长：

$$
P_r'=\mathbf t_r
$$

并且：

$$
\mathbf n_r'=-\kappa_r\mathbf t_r
$$

代入：

$$
P'
=
\mathbf t_r
+
l'\mathbf n_r
-
\kappa_r l\mathbf t_r
$$

合并切向项：

$$
\boxed{
P'
=
(1-\kappa_r l)\mathbf t_r
+
l'\mathbf n_r
}
$$

因此 Candidate 相对 Reference Line 的方向偏角满足：

$$
\Delta\psi
=
\mathrm{atan2}
\left(
l',
1-\kappa_r l
\right)
$$

所以：

$$
\boxed{
\psi
=
\psi_r+
\mathrm{atan2}
\left(
l',
1-\kappa_r l
\right)
}
$$

### C. $P''$ 推导

从：

$$
P'
=
(1-\kappa_r l)\mathbf t_r
+
l'\mathbf n_r
$$

定义：

$$
A=1-\kappa_r l
$$

$$
B=l'
$$

于是：

$$
P'=A\mathbf t_r+B\mathbf n_r
$$

再次对 $s$ 求导：

$$
P''
=
A'\mathbf t_r
+A\mathbf t_r'
+B'\mathbf n_r
+B\mathbf n_r'
$$

先求 $A'$：

$$
A'
=
\frac{d}{ds}(1-\kappa_r l)
$$

由乘积法则：

$$
\frac{d}{ds}(\kappa_r l)
=
\kappa_r'l+\kappa_r l'
$$

所以：

$$
A'
=
-\kappa_r'l-\kappa_r l'
$$

同时：

$$
B'=l''
$$

以及：

$$
\mathbf t_r'=\kappa_r\mathbf n_r
$$

$$
\mathbf n_r'=-\kappa_r\mathbf t_r
$$

全部代入：

$$
P''
=
(-\kappa_r'l-\kappa_r l')\mathbf t_r
+
(1-\kappa_r l)\kappa_r\mathbf n_r
+
l''\mathbf n_r
-
l'\kappa_r\mathbf t_r
$$

合并切向项：

$$
(-\kappa_r'l-\kappa_r l')
-
\kappa_r l'
=
-\kappa_r'l-2\kappa_r l'
$$

合并法向项：

$$
\kappa_r(1-\kappa_r l)+l''
$$

因此：

$$
\boxed{
P''
=
(-\kappa_r'l-2\kappa_r l')\mathbf t_r
+
[\kappa_r(1-\kappa_r l)+l'']\mathbf n_r
}
$$

### D. 平面曲线通用曲率公式

先考虑任意参数 $u$ 的平面曲线：

$$
P(u)=
\begin{bmatrix}
x(u)\\
y(u)
\end{bmatrix}
$$

定义速度模长：

$$
q=\|P'\|
$$

单位切向量可写为：

$$
\mathbf t=
\begin{bmatrix}
\cos\psi\\
\sin\psi
\end{bmatrix}
$$

因此：

$$
P'=q\mathbf t
$$

对 $u$ 再求导：

$$
P''
=
q'\mathbf t
+
q\mathbf t'
$$

而：

$$
\mathbf t'
=
\frac{d\psi}{du}
\begin{bmatrix}
-\sin\psi\\
\cos\psi
\end{bmatrix}
$$

定义单位法向量：

$$
\mathbf n=
\begin{bmatrix}
-\sin\psi\\
\cos\psi
\end{bmatrix}
$$

则：

$$
P''
=
q'\mathbf t
+
q\frac{d\psi}{du}\mathbf n
$$

计算二维叉积：

$$
P'\times P''
=
(q\mathbf t)
\times
\left(
q'\mathbf t
+
q\frac{d\psi}{du}\mathbf n
\right)
$$

展开：

$$
P'\times P''
=
qq'(\mathbf t\times\mathbf t)
+
q^2\frac{d\psi}{du}(\mathbf t\times\mathbf n)
$$

因为：

$$
\mathbf t\times\mathbf t=0
$$

$$
\mathbf t\times\mathbf n=1
$$

所以：

$$
P'\times P''
=
q^2\frac{d\psi}{du}
$$

曲线自身弧长记为 $\ell$，有：

$$
\frac{d\ell}{du}=q
$$

曲率定义：

$$
\kappa
=
\frac{d\psi}{d\ell}
$$

由链式法则：

$$
\frac{d\psi}{d\ell}
=
\frac{d\psi/du}{d\ell/du}
=
\frac{1}{q}\frac{d\psi}{du}
$$

因此：

$$
\frac{d\psi}{du}=q\kappa
$$

代回叉积：

$$
P'\times P''
=
q^3\kappa
$$

又因为：

$$
q=\|P'\|
$$

最终得到：

$$
\boxed{
\kappa
=
\frac{P'\times P''}{\|P'\|^3}
}
$$

若展开为 XY 分量：

$$
P'=
\begin{bmatrix}
x'\\
y'
\end{bmatrix},
\qquad
P''=
\begin{bmatrix}
x''\\
y''
\end{bmatrix}
$$

二维叉积：

$$
P'\times P''
=
x'y''-y'x''
$$

模长：

$$
\|P'\|^3
=
[(x')^2+(y')^2]^{3/2}
$$

所以常见形式为：

$$
\boxed{
\kappa
=
\frac{x'y''-y'x''}
{[(x')^2+(y')^2]^{3/2}}
}
$$

### E. 代入 Frenet 的 $P'$ 和 $P''$

令：

$$
A=1-\kappa_r l,
\qquad
B=l'
$$

$$
C=-\kappa_r'l-2\kappa_r l',
\qquad
D=\kappa_r(1-\kappa_r l)+l''
$$

则：

$$
P'=A\mathbf t_r+B\mathbf n_r
$$

$$
P''=C\mathbf t_r+D\mathbf n_r
$$

因为：

$$
\mathbf t_r\times\mathbf t_r=0
$$

$$
\mathbf n_r\times\mathbf n_r=0
$$

$$
\mathbf t_r\times\mathbf n_r=1
$$

$$
\mathbf n_r\times\mathbf t_r=-1
$$

所以：

$$
P'\times P''
=
(A\mathbf t_r+B\mathbf n_r)
\times
(C\mathbf t_r+D\mathbf n_r)
$$

逐项展开：

$$
P'\times P''
=
AC(\mathbf t_r\times\mathbf t_r)
+AD(\mathbf t_r\times\mathbf n_r)
+BC(\mathbf n_r\times\mathbf t_r)
+BD(\mathbf n_r\times\mathbf n_r)
$$

因此：

$$
P'\times P''
=
AD-BC
$$

代入 $A,B,C,D$：

$$
P'\times P''
=
(1-\kappa_r l)
[\kappa_r(1-\kappa_r l)+l'']
-
l'(-\kappa_r'l-2\kappa_r l')
$$

整理：

$$
P'\times P''
=
(1-\kappa_r l)
[\kappa_r(1-\kappa_r l)+l'']
+
l'(\kappa_r'l+2\kappa_r l')
$$

同时，因为 $\mathbf t_r$ 和 $\mathbf n_r$ 是正交单位向量：

$$
\|P'\|^2
=
A^2+B^2
$$

即：

$$
\|P'\|^2
=
(1-\kappa_r l)^2+(l')^2
$$

所以：

$$
\|P'\|^3
=
[(1-\kappa_r l)^2+(l')^2]^{3/2}
$$

最终：

$$
\boxed{
\kappa=
\frac{
(1-\kappa_r l)[\kappa_r(1-\kappa_r l)+l'']
+
l'(\kappa_r'l+2\kappa_r l')
}{
[(1-\kappa_r l)^2+(l')^2]^{3/2}
}
}
$$
