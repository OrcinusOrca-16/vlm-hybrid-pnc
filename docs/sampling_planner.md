# Sampling Planner

## Goal

在 Frenet / SL 空间生成多条 Candidate：

$$
l=l(s)
$$

通过 constraint / collision / cost 选择 best trajectory。

Candidate 最初表示为：

$$
(s,l,l',l'')
$$

但碰撞检测、曲率约束和 MPC 等最终需要：

$$
(x,y,\psi,\kappa)
$$

因此核心目标：

$$
\boxed{
(s,l,l',l'')
\rightarrow
(x,y,\psi,\kappa)
}
$$

---

## Reference Line

**目的：SL 坐标系本身是建立在 Reference Line 上的，因此要把 Candidate 从 SL 转换到全局 XY，必须从同一 $s$ 处已知的 Reference Line 几何量出发。**

所有带下标 $r$ 的量都属于 Reference Line：

$$
P_r=
\begin{bmatrix}
x_r\\
y_r
\end{bmatrix}
$$

$$
\psi_r
$$

$$
\kappa_r
$$

给定 $s$ 后，可以从 Reference Line 得到这些已知量。

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
\frac{d\mathbf t_r}{ds} =
\kappa_r\mathbf n_r
$$

$$
\frac{d\mathbf n_r}{ds} =
-\kappa_r\mathbf t_r
$$

---

## Frenet Variables

**目的：$l$ 只能描述 Candidate 的横向位置；还需要 $l'$ 描述轨迹方向、$l''$ 描述方向变化，才能进一步求 yaw 和 curvature。**

$$
l=l(s)
$$

表示相对于 Reference Line 的横向位置。

$$
l'=\frac{dl}{ds}
$$

表示横向位置随 $s$ 的变化趋势，主要影响 Candidate 的方向。

$$
l''=\frac{d^2l}{ds^2}
$$

表示 $l'$ 随 $s$ 的变化趋势，主要影响 Candidate 的弯曲程度。

注意：

$$
l' \neq \frac{dl}{dt}
$$

这里都是对 Reference Line 弧长 $s$ 的空间导数。

---

## Position — $P$

**目的：先求 Candidate 在全局 XY 坐标系中的实际位置。**

$P(s)$ 是 Candidate 上的位置向量：

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

**目的：位置 $P$ 不能表示轨迹朝向，因此对 $P$ 求导得到 Candidate 的切线方向，再用它求 yaw。**

$$
P'=\frac{dP}{ds}
$$

得到：

$$
\boxed{
P'
=
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
\psi =
\psi_r+
\operatorname{atan2}
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

**目的：$P'$ 表示方向，$P''$ 表示方向如何变化，用于进一步计算 curvature。**

$$
P''=\frac{d^2P}{ds^2}
$$

其中：

$$
\kappa_r'=\frac{d\kappa_r}{ds}
$$

表示 Reference Line 曲率沿 $s$ 的变化率。

得到：

$$
\boxed{
P'' =
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
\kappa =
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

实际实现中，$\kappa_r'$ 由离散 Reference Line 数值差分得到：

$$
\boxed{
\kappa'_{r,i}
\approx
\frac{\kappa_{r,i+1}-\kappa_{r,i-1}}
{s_{i+1}-s_{i-1}}
}
$$

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
```

最终：

$$
\boxed{
(s,l,l',l'') \rightarrow (x,y,\psi,\kappa)
}
$$