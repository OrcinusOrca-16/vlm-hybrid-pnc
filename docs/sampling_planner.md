# Sampling Planner

## Goal

在 Frenet / SL 空间生成多条 Candidate Path：

$$
l=l(s)
$$

通过 constraint / collision / cost 选择 best path。

Candidate：

$$
(s,l,l',l'')
$$

Cartesian PathPoint：

$$
(x,y,s,\psi,\kappa)
$$

核心转换：

$$
\boxed{(s,l,l',l'')\rightarrow(x,y,s,\psi,\kappa)}
$$

---

## Reference Line

给定 $s$，由 `ReferenceLine.query(s)` 得到：

$$
P_r,\quad \psi_r,\quad \kappa_r,\quad \kappa_r'
$$

其中：

$$
P_r=\begin{bmatrix}x_r\\y_r\end{bmatrix}
$$

单位切向量：

$$
\mathbf{t}_r=\begin{bmatrix}\cos\psi_r\\\sin\psi_r\end{bmatrix}
$$

单位左法向量：

$$
\mathbf{n}_r=\begin{bmatrix}-\sin\psi_r\\\cos\psi_r\end{bmatrix}
$$

并且：

$$
\frac{d\mathbf{t}_r}{ds}=\kappa_r\mathbf{n}_r
$$

$$
\frac{d\mathbf{n}_r}{ds}=-\kappa_r\mathbf{t}_r
$$

当前 $\kappa_r'$ 对 spline 上连续的 $\kappa_r(s)$ 做数值差分：

$$
\kappa_r'(s)\approx\frac{\kappa_r(s+\Delta s)-\kappa_r(s-\Delta s)}{2\Delta s}
$$

端点使用单边差分。

> 以下 Frenet 几何按 $s$ 为 Reference Line 弧长参数推导。当前实现以 raw waypoint 累计距离作为 spline 参数，并将其作为弧长近似。

---

## Frenet Variables

$$
l=l(s)
$$

横向位置。

$$
l'=\frac{dl}{ds}
$$

主要影响 Candidate 方向。

$$
l''=\frac{d^2l}{ds^2}
$$

主要影响 Candidate 曲率。

注意：

$$
l'\neq\frac{dl}{dt}
$$

这里都是对 Reference Line 弧长 $s$ 的空间导数。

---

## Position — $P$

$$
P=P_r+l\mathbf{n}_r
$$

因此：

$$
\boxed{x=x_r-l\sin\psi_r}
$$

$$
\boxed{y=y_r+l\cos\psi_r}
$$

---

## Direction — $P'$

$$
P'=\frac{dP}{ds}
$$

得到：

$$
\boxed{P'=(1-\kappa_r l)\mathbf{t}_r+l'\mathbf{n}_r}
$$

在 Reference Line 局部坐标系中：

$$
P'\Longleftrightarrow\begin{bmatrix}1-\kappa_r l\\l'\end{bmatrix}
$$

因此：

$$
\boxed{\psi=\psi_r+\operatorname{atan2}(l',1-\kappa_r l)}
$$

其中 $1-\kappa_r l$ 表示弯道内 / 外侧造成的切向尺度变化。

---

## Direction Change — $P''$

$$
P''=\frac{d^2P}{ds^2}
$$

其中：

$$
\kappa_r'=\frac{d\kappa_r}{ds}
$$

得到：

$$
\boxed{P''=(-\kappa_r'l-2\kappa_r l')\mathbf{t}_r+[\kappa_r(1-\kappa_r l)+l'']\mathbf{n}_r}
$$

---

## Curvature — $\kappa$

平面曲线通用公式：

$$
\boxed{\kappa=\frac{P'\times P''}{\|P'\|^3}}
$$

代入 Frenet 几何：

$$
\boxed{\kappa=\frac{(1-\kappa_r l)[\kappa_r(1-\kappa_r l)+l'']+l'(\kappa_r'l+2\kappa_r l')}{[(1-\kappa_r l)^2+(l')^2]^{3/2}}}
$$

分母对应：

$$
\|P'\|^2=(1-\kappa_r l)^2+(l')^2
$$

当它接近 $0$ 时 Frenet 几何退化，实际实现需要分母保护。

---

## Main Flow

```text
Frenet Candidate
s, l, l', l''
      ↓
Reference Line
Pr, ψr, κr, κr'
      ↓
P = Pr + l nr
      ↓
x, y
      ↓
P'
      ↓
yaw ψ
      ↓
P''
      ↓
curvature κ
      ↓
Cartesian PathPoint
x, y, s, ψ, κ
```

$$
\boxed{(s,l,l',l'')\rightarrow(x,y,s,\psi,\kappa)}
$$

---

## Appendix — Geometry Derivation

### A. Reference Line 切 / 法向量导数

单位切向量：

$$
\mathbf{t}_r=\begin{bmatrix}\cos\psi_r\\\sin\psi_r\end{bmatrix}
$$

对 $s$ 求导：

$$
\frac{d\mathbf{t}_r}{ds}=\begin{bmatrix}-\sin\psi_r\frac{d\psi_r}{ds}\\\cos\psi_r\frac{d\psi_r}{ds}\end{bmatrix}
$$

弧长参数下：

$$
\kappa_r=\frac{d\psi_r}{ds}
$$

代入：

$$
\frac{d\mathbf{t}_r}{ds}=\kappa_r\begin{bmatrix}-\sin\psi_r\\\cos\psi_r\end{bmatrix}
$$

因此：

$$
\boxed{\frac{d\mathbf{t}_r}{ds}=\kappa_r\mathbf{n}_r}
$$

单位左法向量：

$$
\mathbf{n}_r=\begin{bmatrix}-\sin\psi_r\\\cos\psi_r\end{bmatrix}
$$

对 $s$ 求导：

$$
\frac{d\mathbf{n}_r}{ds}=\begin{bmatrix}-\cos\psi_r\frac{d\psi_r}{ds}\\-\sin\psi_r\frac{d\psi_r}{ds}\end{bmatrix}
$$

代入 $d\psi_r/ds=\kappa_r$：

$$
\frac{d\mathbf{n}_r}{ds}=-\kappa_r\begin{bmatrix}\cos\psi_r\\\sin\psi_r\end{bmatrix}
$$

因此：

$$
\boxed{\frac{d\mathbf{n}_r}{ds}=-\kappa_r\mathbf{t}_r}
$$

### B. $P'$ 推导

Candidate：

$$
P=P_r+l\mathbf{n}_r
$$

对 $s$ 求导，使用乘积法则：

$$
P'=P_r'+l'\mathbf{n}_r+l\mathbf{n}_r'
$$

因为 $s$ 是 Reference Line 弧长：

$$
P_r'=\mathbf{t}_r
$$

并且：

$$
\mathbf{n}_r'=-\kappa_r\mathbf{t}_r
$$

代入：

$$
P'=\mathbf{t}_r+l'\mathbf{n}_r-\kappa_r l\mathbf{t}_r
$$

合并切向项：

$$
\boxed{P'=(1-\kappa_r l)\mathbf{t}_r+l'\mathbf{n}_r}
$$

Candidate 相对 Reference Line 的方向偏角：

$$
\Delta\psi=\operatorname{atan2}(l',1-\kappa_r l)
$$

因此：

$$
\boxed{\psi=\psi_r+\operatorname{atan2}(l',1-\kappa_r l)}
$$

### C. $P''$ 推导

从：

$$
P'=(1-\kappa_r l)\mathbf{t}_r+l'\mathbf{n}_r
$$

定义：

$$
A=1-\kappa_r l,\qquad B=l'
$$

则：

$$
P'=A\mathbf{t}_r+B\mathbf{n}_r
$$

再次求导：

$$
P''=A'\mathbf{t}_r+A\mathbf{t}_r'+B'\mathbf{n}_r+B\mathbf{n}_r'
$$

先求 $A'$：

$$
A'=\frac{d}{ds}(1-\kappa_r l)
$$

乘积法则：

$$
\frac{d}{ds}(\kappa_r l)=\kappa_r'l+\kappa_r l'
$$

所以：

$$
A'=-\kappa_r'l-\kappa_r l'
$$

同时：

$$
B'=l''
$$

并且：

$$
\mathbf{t}_r'=\kappa_r\mathbf{n}_r,\qquad \mathbf{n}_r'=-\kappa_r\mathbf{t}_r
$$

全部代入：

$$
P''=(-\kappa_r'l-\kappa_r l')\mathbf{t}_r+(1-\kappa_r l)\kappa_r\mathbf{n}_r+l''\mathbf{n}_r-l'\kappa_r\mathbf{t}_r
$$

切向项：

$$
(-\kappa_r'l-\kappa_r l')-\kappa_r l'=-\kappa_r'l-2\kappa_r l'
$$

法向项：

$$
\kappa_r(1-\kappa_r l)+l''
$$

因此：

$$
\boxed{P''=(-\kappa_r'l-2\kappa_r l')\mathbf{t}_r+[\kappa_r(1-\kappa_r l)+l'']\mathbf{n}_r}
$$

### D. 平面曲线通用曲率公式

考虑任意参数 $u$ 的平面曲线：

$$
P(u)=\begin{bmatrix}x(u)\\y(u)\end{bmatrix}
$$

定义：

$$
q=\|P'\|
$$

单位切向量：

$$
\mathbf{t}=\begin{bmatrix}\cos\psi\\\sin\psi\end{bmatrix}
$$

因此：

$$
P'=q\mathbf{t}
$$

再次求导：

$$
P''=q'\mathbf{t}+q\mathbf{t}'
$$

而：

$$
\mathbf{t}'=\frac{d\psi}{du}\mathbf{n}
$$

所以：

$$
P''=q'\mathbf{t}+q\frac{d\psi}{du}\mathbf{n}
$$

计算二维叉积：

$$
P'\times P''=(q\mathbf{t})\times\left(q'\mathbf{t}+q\frac{d\psi}{du}\mathbf{n}\right)
$$

展开：

$$
P'\times P''=qq'(\mathbf{t}\times\mathbf{t})+q^2\frac{d\psi}{du}(\mathbf{t}\times\mathbf{n})
$$

因为：

$$
\mathbf{t}\times\mathbf{t}=0,\qquad \mathbf{t}\times\mathbf{n}=1
$$

所以：

$$
P'\times P''=q^2\frac{d\psi}{du}
$$

曲线自身弧长记为 $\ell$：

$$
\frac{d\ell}{du}=q
$$

曲率定义：

$$
\kappa=\frac{d\psi}{d\ell}
$$

链式法则：

$$
\kappa=\frac{d\psi/du}{d\ell/du}=\frac{1}{q}\frac{d\psi}{du}
$$

因此：

$$
\frac{d\psi}{du}=q\kappa
$$

代回：

$$
P'\times P''=q^3\kappa
$$

又因为 $q=\|P'\|$：

$$
\boxed{\kappa=\frac{P'\times P''}{\|P'\|^3}}
$$

若展开为 XY 分量：

$$
P'=\begin{bmatrix}x'\\y'\end{bmatrix},\qquad P''=\begin{bmatrix}x''\\y''\end{bmatrix}
$$

二维叉积：

$$
P'\times P''=x'y''-y'x''
$$

模长：

$$
\|P'\|^3=[(x')^2+(y')^2]^{3/2}
$$

因此：

$$
\boxed{\kappa=\frac{x'y''-y'x''}{[(x')^2+(y')^2]^{3/2}}}
$$

### E. 代入 Frenet 的 $P'$ 和 $P''$

定义：

$$
A=1-\kappa_r l,\quad B=l',\quad C=-\kappa_r'l-2\kappa_r l',\quad D=\kappa_r(1-\kappa_r l)+l''
$$

于是：

$$
P'=A\mathbf{t}_r+B\mathbf{n}_r
$$

$$
P''=C\mathbf{t}_r+D\mathbf{n}_r
$$

叉积展开：

$$
P'\times P''=(A\mathbf{t}_r+B\mathbf{n}_r)\times(C\mathbf{t}_r+D\mathbf{n}_r)
$$

$$
P'\times P''=AC(\mathbf{t}_r\times\mathbf{t}_r)+AD(\mathbf{t}_r\times\mathbf{n}_r)+BC(\mathbf{n}_r\times\mathbf{t}_r)+BD(\mathbf{n}_r\times\mathbf{n}_r)
$$

利用：

$$
\mathbf{t}_r\times\mathbf{t}_r=0,\quad \mathbf{n}_r\times\mathbf{n}_r=0,\quad \mathbf{t}_r\times\mathbf{n}_r=1,\quad \mathbf{n}_r\times\mathbf{t}_r=-1
$$

得到：

$$
P'\times P''=AD-BC
$$

代入 $A,B,C,D$：

$$
P'\times P''=(1-\kappa_r l)[\kappa_r(1-\kappa_r l)+l'']+l'(\kappa_r'l+2\kappa_r l')
$$

因为 $\mathbf{t}_r$、$\mathbf{n}_r$ 是正交单位向量：

$$
\|P'\|^2=A^2+B^2=(1-\kappa_r l)^2+(l')^2
$$

所以：

$$
\|P'\|^3=[(1-\kappa_r l)^2+(l')^2]^{3/2}
$$

最终：

$$
\boxed{\kappa=\frac{(1-\kappa_r l)[\kappa_r(1-\kappa_r l)+l'']+l'(\kappa_r'l+2\kappa_r l')}{[(1-\kappa_r l)^2+(l')^2]^{3/2}}}
$$
