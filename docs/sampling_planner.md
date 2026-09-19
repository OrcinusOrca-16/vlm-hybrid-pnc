# Sampling Planner

## Goal

从当前 ego state 出发，在 Frenet / SL 空间生成多条 Candidate Path：

$$
l=l(s)
$$

再转换回 Cartesian，供后续 constraint / collision / cost 选择 best path。

### Main Flow

```text
VehicleState
x, y, ψ, κ
      ↓
Cartesian → Frenet
      ↓
initial Frenet state
s, l, l', l''
      ↓
sample target l + target length
      ↓
quintic polynomial
      ↓
multiple Frenet Candidates
(s, l, l', l'')
      ↓
Frenet → Cartesian geometry
      ├─ P        → x, y
      ├─ P'       → yaw ψ
      └─ P', P''  → curvature κ
      ↓
Cartesian Candidate Paths
(x, y, path_s, ψ, κ)
```

核心链路：

$$
\boxed{
(x,y,\psi,\kappa)
\rightarrow
(s,l,l',l'')
\rightarrow
\text{Frenet Candidates}
\rightarrow
(x,y,s,\psi,\kappa)
}
$$

---

## Reference Line

给定 $s$，由 \`ReferenceLine.query(s)\` 得到：

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

## 1. Ego State → Initial Frenet State

当前 ego Cartesian 几何状态：

$$
(x,y,\psi,\kappa)
$$

先通过 XY → SL 投影得到：

$$
(s,l)
$$

再查询 Reference Line 在 $s$ 处的：

$$
\psi_r,\quad \kappa_r,\quad \kappa_r'
$$

定义 heading difference：

$$
\Delta\psi=\psi-\psi_r
$$

由：

$$
P'=(1-\kappa_r l)\mathbf{t}_r+l'\mathbf{n}_r
$$

可知 $P'$ 在 Reference Line 局部坐标系中的切向 / 法向分量为：

$$
1-\kappa_r l,\qquad l'
$$

因此：

$$
\tan\Delta\psi=\frac{l'}{1-\kappa_r l}
$$

得到：

$$
\boxed{l'=(1-\kappa_r l)\tan\Delta\psi}
$$

定义：

$$
A=1-\kappa_r l
$$

则：

$$
\boxed{
l''=-(\kappa_r'l+\kappa_r l')\tan\Delta\psi
+\frac{A}{\cos^2\Delta\psi}
\left[
\kappa\frac{A}{\cos\Delta\psi}
-\kappa_r
\right]
}
$$

其中用到：

$$
\boxed{\kappa=\frac{d\psi}{ds_{\mathrm{ego}}}}
$$

以及：

$$
\boxed{
\frac{ds_{\mathrm{ego}}}{ds}
=\frac{1-\kappa_r l}{\cos\Delta\psi}
}
$$

直觉：

```text
l'  → 当前 heading 相对 Reference Line 偏了多少
l'' → 这个相对几何接下来如何变化
      → ego curvature κ
      → reference curvature κr
      → reference curvature derivative κr'
```

Sanity check：

若 ego 正好位于 Reference Line 上，并且 heading / curvature 相同：

$$
l=0,\qquad \Delta\psi=0,\qquad \kappa=\kappa_r
$$

则：

$$
l'=0,\qquad l''=0
$$

> 角度统一使用 rad；曲率单位为 $1/m$。详细推导见 Appendix B。

---

## 2. Lateral Sampling

当前第一版只做空间 Path Sampling，不进行 longitudinal trajectory optimization。

Target lateral positions：

$$
l_f\in\{-1.5,-0.75,0,0.75,1.5\}\;m
$$

Target path lengths：

$$
S\in\{10,20\}\;m
$$

Sampling resolution：

$$
\Delta s=0.5\;m
$$

因此当前共生成：

$$
5\times2=10
$$

条 Candidate。

同样的横向目标下，$S$ 越长，横移可以完成得越平缓。

### Quintic Polynomial

令局部纵向变量：

$$
\sigma=s-s_0,\qquad 0\le\sigma\le S
$$

横向曲线：

$$
l(\sigma)=a_0+a_1\sigma+a_2\sigma^2+a_3\sigma^3+a_4\sigma^4+a_5\sigma^5
$$

六个系数对应六个边界条件：

$$
l_0,\quad l_0',\quad l_0''
$$

以及：

$$
l_f,\quad l_f',\quad l_f''
$$

当前 terminal state：

$$
l_f'=0,\qquad l_f''=0
$$

所以每组 $(l_f,S)$ 生成一条 smooth Frenet Candidate：

$$
(s,l,l',l'')
$$

再按 $\Delta s=0.5\;m$ 离散采样。

---

## 3. Frenet Candidate → Cartesian Path

Quintic Sampling 得到的是：

$$
(s,l,l',l'')
$$

后续 obstacle / footprint / curvature constraint 使用的是 Cartesian PathPoint：

$$
(x,y,\mathrm{path\_s},\psi,\kappa)
$$

所以需要把每个 Frenet Candidate 转回 Cartesian geometry。

### Position — $P$

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

即：

```text
P → x, y
```

### Direction — $P'$

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

即：

```text
P' → yaw ψ
```

其中 $1-\kappa_r l$ 表示弯道内 / 外侧造成的切向尺度变化。

### Direction Change — $P''$

$$
P''=\frac{d^2P}{ds^2}
$$

由 $P'$ 再求导：

$$
P''=\frac{d}{ds}\left[(1-\kappa_r l)\mathbf{t}_r+l'\mathbf{n}_r\right]
$$

整理切向 / 法向项：

$$
\boxed{
P''=(-\kappa_r'l-2\kappa_r l')\mathbf{t}_r
+[\kappa_r(1-\kappa_r l)+l'']\mathbf{n}_r
}
$$

### Curvature — $\kappa$

平面曲线通用公式：

$$
\boxed{\kappa=\frac{P'\times P''}{\lVert P'\rVert^3}}
$$

代入 Frenet 几何：

$$
\boxed{
\kappa=
\frac{
(1-\kappa_r l)[\kappa_r(1-\kappa_r l)+l'']
+l'(\kappa_r'l+2\kappa_r l')
}{
[(1-\kappa_r l)^2+(l')^2]^{3/2}
}
}
$$

即：

```text
P', P'' → curvature κ
```

分母对应：

$$
\lVert P'\rVert^2=(1-\kappa_r l)^2+(l')^2
$$

当它接近 $0$ 时 Frenet 几何退化，实际实现需要分母保护。

> \`FrenetTrajectoryPoint.s\` 是 Reference Line station；\`PathPoint.s\` 是 Candidate Cartesian path 自己的累计弧长，两者语义不同。

---

## Appendix — Geometry Derivation

### A. Reference Line 切 / 法向量导数

单位切向量：

$$
\mathbf{t}_r=(\cos\psi_r,\sin\psi_r)^T
$$

对 $s$ 求导：

$$
\frac{d\mathbf{t}_r}{ds}
=
\left(
-\sin\psi_r\frac{d\psi_r}{ds},
\ \cos\psi_r\frac{d\psi_r}{ds}
\right)^T
$$

弧长参数下：

$$
\kappa_r=\frac{d\psi_r}{ds}
$$

因此：

$$
\boxed{\frac{d\mathbf{t}_r}{ds}=\kappa_r\mathbf{n}_r}
$$

单位左法向量：

$$
\mathbf{n}_r=(-\sin\psi_r,\cos\psi_r)^T
$$

同理：

$$
\boxed{\frac{d\mathbf{n}_r}{ds}=-\kappa_r\mathbf{t}_r}
$$

### B. Ego Cartesian → Initial Frenet $l'$ / $l''$

从：

$$
P=P_r+l\mathbf{n}_r
$$

可得：

$$
P'=(1-\kappa_r l)\mathbf{t}_r+l'\mathbf{n}_r
$$

定义：

$$
\Delta\psi=\psi-\psi_r
$$

因为 $P'$ 在 Reference Line 局部坐标系中的切向 / 法向分量分别是：

$$
1-\kappa_r l,\qquad l'
$$

所以：

$$
\tan\Delta\psi=\frac{l'}{1-\kappa_r l}
$$

得到：

$$
\boxed{l'=(1-\kappa_r l)\tan\Delta\psi}
$$

定义：

$$
A=1-\kappa_r l
$$

则：

$$
l'=A\tan\Delta\psi
$$

对 Reference Line 的 $s$ 再求导：

$$
l''
=
A'\tan\Delta\psi
+
A\sec^2\Delta\psi
\frac{d\Delta\psi}{ds}
$$

其中：

$$
A'=-(\kappa_r'l+\kappa_r l')
$$

由于：

$$
\Delta\psi=\psi-\psi_r
$$

因此：

$$
\frac{d\Delta\psi}{ds}
=
\frac{d\psi}{ds}-\kappa_r
$$

Ego 曲率定义在 ego 自己的路径弧长 $s_{\mathrm{ego}}$ 上：

$$
\boxed{\kappa=\frac{d\psi}{ds_{\mathrm{ego}}}}
$$

Ego 的微小位移 $ds_{\mathrm{ego}}$ 在 Reference Line 切向上的投影为：

$$
ds_{\mathrm{ego}}\cos\Delta\psi
$$

另一方面，由：

$$
dP=P'\,ds
$$

可知其 Reference Line 切向位移为：

$$
A\,ds
$$

所以：

$$
ds_{\mathrm{ego}}\cos\Delta\psi=A\,ds
$$

因此：

$$
\boxed{
\frac{ds_{\mathrm{ego}}}{ds}
=
\frac{A}{\cos\Delta\psi}
}
$$

由链式法则：

$$
\frac{d\psi}{ds}
=
\frac{d\psi}{ds_{\mathrm{ego}}}
\frac{ds_{\mathrm{ego}}}{ds}
=
\kappa\frac{A}{\cos\Delta\psi}
$$

所以：

$$
\frac{d\Delta\psi}{ds}
=
\kappa\frac{A}{\cos\Delta\psi}-\kappa_r
$$

最终：

$$
\boxed{
l''
=
-(\kappa_r'l+\kappa_r l')\tan\Delta\psi
+
\frac{A}{\cos^2\Delta\psi}
\left[
\kappa\frac{A}{\cos\Delta\psi}
-\kappa_r
\right]
}
$$

其中：

$$
A=1-\kappa_r l
$$

### C. $P'$ 推导

Candidate：

$$
P=P_r+l\mathbf{n}_r
$$

对 $s$ 求导，使用乘积法则：

$$
P'=P_r'+l'\mathbf{n}_r+l\mathbf{n}_r'
$$

因为：

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

因此：

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

### D. $P''$ 推导

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
P''
=
A'\mathbf{t}_r
+
A\mathbf{t}_r'
+
B'\mathbf{n}_r
+
B\mathbf{n}_r'
$$

其中：

$$
A'=-(\kappa_r'l+\kappa_r l'),\qquad B'=l''
$$

并且：

$$
\mathbf{t}_r'=\kappa_r\mathbf{n}_r,\qquad
\mathbf{n}_r'=-\kappa_r\mathbf{t}_r
$$

整理得到：

$$
\boxed{
P''=
(-\kappa_r'l-2\kappa_r l')\mathbf{t}_r
+
[\kappa_r(1-\kappa_r l)+l'']\mathbf{n}_r
}
$$

### E. 平面曲线通用曲率公式

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
P'\times P''
=
q^2\frac{d\psi}{du}
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
\kappa
=
\frac{d\psi/du}{d\ell/du}
=
\frac{1}{q}\frac{d\psi}{du}
$$

因此：

$$
\boxed{\kappa=\frac{P'\times P''}{\lVert P'\rVert^3}}
$$

若展开为 XY 分量：

$$
\boxed{
\kappa=
\frac{x'y''-y'x''}
{[(x')^2+(y')^2]^{3/2}}
}
$$

### F. 代入 Frenet 的 $P'$ 和 $P''$

定义：

$$
A=1-\kappa_r l,\quad
B=l',\quad
C=-\kappa_r'l-2\kappa_r l',\quad
D=\kappa_r(1-\kappa_r l)+l''
$$

于是：

$$
P'=A\mathbf{t}_r+B\mathbf{n}_r
$$

$$
P''=C\mathbf{t}_r+D\mathbf{n}_r
$$

利用：

$$
\mathbf{t}_r\times\mathbf{t}_r=0,\qquad
\mathbf{n}_r\times\mathbf{n}_r=0
$$

$$
\mathbf{t}_r\times\mathbf{n}_r=1,\qquad
\mathbf{n}_r\times\mathbf{t}_r=-1
$$

得到：

$$
P'\times P''=AD-BC
$$

代入 $A,B,C,D$：

$$
P'\times P''
=
(1-\kappa_r l)[\kappa_r(1-\kappa_r l)+l'']
+
l'(\kappa_r'l+2\kappa_r l')
$$

同时：

$$
\lVert P'\rVert^2
=
(1-\kappa_r l)^2+(l')^2
$$

最终：

$$
\boxed{
\kappa=
\frac{
(1-\kappa_r l)[\kappa_r(1-\kappa_r l)+l'']
+l'(\kappa_r'l+2\kappa_r l')
}{
[(1-\kappa_r l)^2+(l')^2]^{3/2}
}
}
$$
