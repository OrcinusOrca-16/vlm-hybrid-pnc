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
P_r=(x_r,y_r)^T
$$

单位切向量：

$$
\mathbf{t}_r=(\cos\psi_r,\sin\psi_r)^T
$$

单位左法向量：

$$
\mathbf{n}_r=(-\sin\psi_r,\cos\psi_r)^T
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

## Cartesian Ego State → Initial Frenet State

Sampling 从当前 ego Cartesian 几何状态开始：

$
(x,y,\psi,\kappa)
$

先通过 XY → SL 投影得到：

$
(s,l)
$

再查询 Reference Line 在 $s$ 处的：

$
\psi_r,\quad \kappa_r,\quad \kappa_r'
$

定义 ego 与 Reference Line 的方向差：

$
\Delta\psi=\psi-\psi_r
$

实际实现将 $\Delta\psi$ wrap 到 $(-\pi,\pi]$。

由前面的：

$
P'=(1-\kappa_r l)\mathbf{t}_r+l'\mathbf{n}_r
$

可知 $P'$ 在 Reference Line 局部坐标系中的切向 / 法向分量为：

$
1-\kappa_r l,qquad l'
$

因此：

$
\tan\Delta\psi=
\frac{l'}{1-\kappa_r l}
$

得到：

$
\boxed{
l'=(1-\kappa_r l)\tan\Delta\psi
}
$

定义：

$
A=1-\kappa_r l
$

则：

$
l'=A\tan\Delta\psi
$

对 Reference Line 的 $s$ 再求导：

$
l''
=
A'\tan\Delta\psi
+
A\sec^2\Delta\psi
\frac{d\Delta\psi}{ds}
$

其中：

$
A'=-(\kappa_r'l+\kappa_r l')
$

Ego 曲率定义在 ego 自己的路径弧长 $s_{\mathrm{ego}}$ 上：

$
\boxed{
\kappa=\frac{d\psi}{ds_{\mathrm{ego}}}
}
$

Ego 的微小位移 $ds_{\mathrm{ego}}$ 投影到 Reference Line 切向：

$
ds_{\mathrm{ego}}\cos\Delta\psi=A\,ds
$

因此：

$
\boxed{
\frac{ds_{\mathrm{ego}}}{ds}
=
\frac{A}{\cos\Delta\psi}
}
$

又因为：

$
\Delta\psi=\psi-\psi_r
$

所以：

$
\frac{d\Delta\psi}{ds}
=
\kappa\frac{A}{\cos\Delta\psi}
-
\kappa_r
$

最终：

$
\boxed{
l''
=
-(\kappa_r'l+\kappa_r l')\tan\Delta\psi
+
\frac{A}{\cos^2\Delta\psi}
\left[
\kappa\frac{A}{\cos\Delta\psi}
-
\kappa_r
\right]
}
$

其中：

$
A=1-\kappa_r l
$

直觉上：

- $l'$ 主要表示当前 heading 相对 Reference Line 的偏离；
- $l''$ 表示这种相对几何接下来如何变化，因此会包含 ego 曲率 $\kappa$、Reference Line 曲率 $\kappa_r$ 和曲率变化 $\kappa_r'$。

Sanity check：如果 ego 正好位于 Reference Line 上，并且 heading / curvature 都相同：

$
l=0,\quad \Delta\psi=0,\quad \kappa=\kappa_r
$

则：

$
l'=0,\qquad l''=0
$

> 以上角度统一使用 rad；曲率单位为 $1/m$。当 $\cos\Delta\psi$ 接近 $0$ 时，Frenet 表达接近奇异，实际实现需要保护。

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
P'\Longleftrightarrow(1-\kappa_r l,l')^T
$$

因此：

$$
\boxed{\psi=\psi_r+\mathrm{atan2}(l',1-\kappa_r l)}
$$

其中 $1-\kappa_r l$ 表示弯道内 / 外侧造成的切向尺度变化。

---

## Direction Change — $P''$

$$
P''=\frac{d^2P}{ds^2}
$$

由 $P'$ 再求导：

$$
P''=\frac{d}{ds}\left[(1-\kappa_r l)\mathbf{t}_r+l'\mathbf{n}_r\right]
$$

其中使用：

$$
(\kappa_r l)'=\kappa_r'l+\kappa_r l'
$$

$$
\mathbf{t}_r'=\kappa_r\mathbf{n}_r,\qquad \mathbf{n}_r'=-\kappa_r\mathbf{t}_r
$$

整理切向 / 法向项：

$$
\boxed{P''=(-\kappa_r'l-2\kappa_r l')\mathbf{t}_r+[\kappa_r(1-\kappa_r l)+l'']\mathbf{n}_r}
$$

---

## Curvature — $\kappa$

平面曲线通用公式：

$$
\boxed{\kappa=\frac{P'\times P''}{\lVert P'\rVert^3}}
$$

代入 Frenet 几何：

$$
\boxed{\kappa=\frac{(1-\kappa_r l)[\kappa_r(1-\kappa_r l)+l'']+l'(\kappa_r'l+2\kappa_r l')}{[(1-\kappa_r l)^2+(l')^2]^{3/2}}}
$$

分母对应：

$$
\lVert P'\rVert^2=(1-\kappa_r l)^2+(l')^2
$$

当它接近 $0$ 时 Frenet 几何退化，实际实现需要分母保护。

---

## Lateral Sampling

当前第一版只做空间 Path Sampling，不进行 longitudinal trajectory optimization。

Target lateral positions：

$
l_f\in\{-1.5,-0.75,0,0.75,1.5\}\;m
$

Target path lengths（Reference Line $s$ 方向的 horizon）：

$
S\in\{10,20\}\;m
$

Sampling resolution：

$
\Delta s=0.5\;m
$

因此当前共生成：

$
5\times2=10
$

条 Candidate。

同样的横向目标下，$S$ 越长，横移可以完成得越平缓。

### Quintic Polynomial

令局部纵向变量：

$
\sigma=s-s_0,\qquad 0\le\sigma\le S
$

横向曲线使用五次多项式：

$
l(\sigma)=
a_0+a_1\sigma+a_2\sigma^2+a_3\sigma^3+a_4\sigma^4+a_5\sigma^5
$

六个系数对应六个边界条件：

$
l_0,\quad l_0',\quad l_0''
$

以及：

$
l_f,\quad l_f',\quad l_f''
$

当前 terminal state 设为：

$
l_f'=0,\qquad l_f''=0
$

每组 $(l_f,S)$ 生成一条 Frenet Candidate：

$
(s,l,l',l'')
$

再按 $\Delta s=0.5\;m$ 离散采样，并转换到 Cartesian Path。

> `FrenetTrajectoryPoint.s` 是 Reference Line station；`PathPoint.s` 是 Candidate Cartesian path 自己的累计弧长，两者语义不同。

---

## Main Flow

```text
VehicleState
x, y, ψ, κ
      ↓
XY → SL
      ↓
initial Frenet state
s, l, l', l''
      ↓
sample target l + target length
      ↓
quintic polynomial
      ↓
multiple Frenet Candidates
      ↓
Reference Line
Pr, ψr, κr, κr'
      ↓
P / P' / P''
      ↓
Cartesian Candidate Paths
x, y, s, ψ, κ
```

$
\boxed{
(x,y,\psi,\kappa)
\rightarrow
(s,l,l',l'')
\rightarrow
\text{Frenet Candidates}
\rightarrow
\text{Cartesian Paths}
}
$

---

## Appendix — Geometry Derivation

### A. Reference Line 切 / 法向量导数

单位切向量：

$$
\mathbf{t}_r=(\cos\psi_r,\sin\psi_r)^T
$$

对 $s$ 求导：

$$
\frac{d\mathbf{t}_r}{ds}=\left(-\sin\psi_r\frac{d\psi_r}{ds},\ \cos\psi_r\frac{d\psi_r}{ds}\right)^T
$$

弧长参数下：

$$
\kappa_r=\frac{d\psi_r}{ds}
$$

代入：

$$
\frac{d\mathbf{t}_r}{ds}=\kappa_r(-\sin\psi_r,\cos\psi_r)^T
$$

因此：

$$
\boxed{\frac{d\mathbf{t}_r}{ds}=\kappa_r\mathbf{n}_r}
$$

单位左法向量：

$$
\mathbf{n}_r=(-\sin\psi_r,\cos\psi_r)^T
$$

对 $s$ 求导：

$$
\frac{d\mathbf{n}_r}{ds}=\left(-\cos\psi_r\frac{d\psi_r}{ds},\ -\sin\psi_r\frac{d\psi_r}{ds}\right)^T
$$

代入 $d\psi_r/ds=\kappa_r$：

$$
\frac{d\mathbf{n}_r}{ds}=-\kappa_r(\cos\psi_r,\sin\psi_r)^T
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
\Delta\psi=\mathrm{atan2}(l',1-\kappa_r l)
$$

因此：

$$
\boxed{\psi=\psi_r+\mathrm{atan2}(l',1-\kappa_r l)}
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
P(u)=(x(u),y(u))^T
$$

定义：

$$
q=\lVert P'\rVert
$$

单位切向量：

$$
\mathbf{t}=(\cos\psi,\sin\psi)^T
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

又因为 $q=\lVert P'\rVert$：

$$
\boxed{\kappa=\frac{P'\times P''}{\lVert P'\rVert^3}}
$$

若展开为 XY 分量：

$$
P'=(x',y')^T,\qquad P''=(x'',y'')^T
$$

二维叉积：

$$
P'\times P''=x'y''-y'x''
$$

模长：

$$
\lVert P'\rVert^3=[(x')^2+(y')^2]^{3/2}
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

因为切向量和法向量是正交单位向量：

$$
\lVert P'\rVert^2=A^2+B^2=(1-\kappa_r l)^2+(l')^2
$$

所以：

$$
\lVert P'\rVert^3=[(1-\kappa_r l)^2+(l')^2]^{3/2}
$$

最终：

$$
\boxed{\kappa=\frac{(1-\kappa_r l)[\kappa_r(1-\kappa_r l)+l'']+l'(\kappa_r'l+2\kappa_r l')}{[(1-\kappa_r l)^2+(l')^2]^{3/2}}}
$$