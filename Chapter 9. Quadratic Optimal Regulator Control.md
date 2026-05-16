1

# Chapter 9. Linear Quadratic Regulators (LQR) Control



## Content

1. Quadratic Optimal Regulator

2. Practical Examples

3. Q-Augmented LQR Controller

4. Selection of Weighting Matrices

2

3

# 1. Quadratic Optimal Regulator

**Quadratic Optimal Problem:**

- Consider a system: $\mathbf{\dot{x}} = \mathbf{Ax} + \mathbf{Bu}$
![Control system block diagram showing feedback loop with gain -K](page_2_image_1_v2.jpg)

The optimal regulator problem is to determine the matrix K of the optimal control vector $\mathbf{u}(t) = -\mathbf{Kx}(t)$ so as to minimize the performance index:

$$ J = \int_{0}^{\infty} (\mathbf{x^*Qx} + \mathbf{u^*Ru}) \, dt \quad \text{(energy expenditure of the system)} $$

where **Q**, **R**: positive-definite Hermitian or real symmetric matrices



# 1. Quadratic Optimal Regulator

**Quadratic Optimal Problem:**

- To find the optimal feedback gain **K**, solve the Algebraic Riccati Equation:
![Control system block diagram showing feedback loop with gain -K](page_2_image_2_v2.jpg)

$$ \mathbf{A'P} + \mathbf{PA} - \mathbf{PBR^{-1}B'P} + \mathbf{Q} = \mathbf{0} $$

Then

$$ \mathbf{K} = \mathbf{R^{-1}B^TP} $$

4

5

# 2. Practical Examples

* **Example 1:** Consider the system described by

$$ \dot{\mathbf{x}} = \mathbf{Ax} + \mathbf{Bu} $$

where

$$ \mathbf{A} = \begin{bmatrix} 0 & 1 \\ 0 & -1 \end{bmatrix}, \quad \mathbf{B} = \begin{bmatrix} 0 \\ 1 \end{bmatrix} $$

The performance index $J$ is given by

$$ J = \int_{0}^{\infty} (\mathbf{x}'\mathbf{Qx} + u'Ru) \, dt $$

where

$$ \mathbf{Q} = \begin{bmatrix} 1 & 0 \\ 0 & 1 \end{bmatrix}, \quad R = [1] $$

Assume that the following control $u$ is used.

$$ u = -\mathbf{Kx} $$

Determine the optimal feedback gain matrix **K**.



# 2. Practical Examples

## Example 1: (solution)

- The optimal feedback gain matrix **K** can be obtained by solving the following Riccati equation for a positive-definite matrix **P**:

$$ \mathbf{A}'\mathbf{P} + \mathbf{PA} - \mathbf{PBR}^{-1}\mathbf{B}'\mathbf{P} + \mathbf{Q} = \mathbf{0} $$

- The result is

$$ \mathbf{P} = \begin{bmatrix} 2 & 1 \\ 1 & 1 \end{bmatrix} $$

- Substituting this **P** matrix into the following equation gives the optimal **K** matrix:

### Matlab

$$ \mathbf{K} = R^{-1}\mathbf{B}'\mathbf{P} $$

$$ = [1][0 \quad 1] \begin{bmatrix} 2 & 1 \\ 1 & 1 \end{bmatrix} = [1 \quad 1] $$

```matlab
A = [0 1;0 -1];
B = [0;1];
Q = [1 0; 0 1];
R = [1];
K = lqr(A,B,Q,R)
K =
    1.0000    1.0000
```

- Thus, the optimal control signal is given by

$$ u = -\mathbf{Kx} = -x_1 - x_2 $$

6

# 2. Practical Examples

* **Example 2:** Consider a plant with state-space equations below. Determine an optimal control law?

![Control system block diagram](page_4_image_2_v2.jpg)

$$ \begin{aligned} \dot{\mathbf{x}} &= \mathbf{Ax} + \mathbf{Bu} \\ y &= \mathbf{Cx} + Du \end{aligned} $$

$$ \text{where} \quad \mathbf{A} = \begin{bmatrix} 0 & 1 & 0 \\ 0 & 0 & 1 \\ 0 & -2 & -3 \end{bmatrix}, $$

$$ \mathbf{B} = \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}, $$

$$ \mathbf{C} = \begin{bmatrix} 1 & 0 & 0 \end{bmatrix}, \quad D = [0] $$

‒ The control signal $u$ is given by:

$$ u = k_1(r - x_1) - (k_2x_2 + k_3x_3) = k_1r - (k_1x_1 + k_2x_2 + k_3x_3) $$


y = x1
<table>
  <tbody>
    <tr>
        <td>Time</td>
        <td>y = x1</td>
    </tr>
    <tr>
        <td>0</td>
        <td>0</td>
    </tr>
    <tr>
        <td>2</td>
        <td>20</td>
    </tr>
    <tr>
        <td>4</td>
        <td>100</td>
    </tr>
    <tr>
        <td>6</td>
        <td>300</td>
    </tr>
    <tr>
        <td>8</td>
        <td>600</td>
    </tr>
    <tr>
        <td>10</td>
        <td>1000</td>
    </tr>
  </tbody>
</table>




# 2. Practical Examples

* **Example 2: (solution)**

‒ State-feedback gain matrix: $\mathbf{K} = \begin{bmatrix} k_1 & k_2 & k_3 \end{bmatrix} = \begin{bmatrix} 100 & 53.12 & 11.6711 \end{bmatrix}$

$$ \text{with} \quad \mathbf{x} = \begin{bmatrix} x_1 \\ x_2 \\ x_3 \end{bmatrix} = \begin{bmatrix} y \\ \dot{y} \\ \ddot{y} \end{bmatrix}, \quad \mathbf{Q} = \begin{bmatrix} 100 & 0 & 0 \\ 0 & 1 & 0 \\ 0 & 0 & 1 \end{bmatrix}, \quad R = 0.01 $$

‒ Step-response characteristics: $\dot{\mathbf{x}} = \mathbf{Ax} + \mathbf{Bu}$

$$ = \mathbf{Ax} + \mathbf{B}(-\mathbf{Kx} + k_1r) $$

$$ = (\mathbf{A} - \mathbf{BK})\mathbf{x} + \mathbf{Bk}_1r $$

Command: `[y,x,t] = step(AA,BB,CC,DD)`

with $\mathbf{AA} = \mathbf{A} - \mathbf{BK}, \quad \mathbf{BB} = \mathbf{Bk}_1, \quad \mathbf{CC} = \mathbf{C}, \quad \mathbf{DD} = D$

7
8

# 2. Practical Examples

* **Example 2: (*solution*)**


Response Curves x1, x2, x3, versus t
<table>
  <tbody>
    <tr>
        <td>t Sec</td>
        <td>0</td>
        <td>1</td>
        <td>2</td>
        <td>3</td>
        <td>4</td>
        <td>5</td>
        <td>6</td>
        <td>7</td>
        <td>8</td>
    </tr>
    <tr>
        <td>x1</td>
        <td>0</td>
        <td>0.8</td>
        <td>1.1</td>
        <td>1.1</td>
        <td>1.1</td>
        <td>1.1</td>
        <td>1.1</td>
        <td>1.1</td>
        <td>1.1</td>
    </tr>
    <tr>
        <td>x2</td>
        <td>0</td>
        <td>0.8</td>
        <td>0.1</td>
        <td>0</td>
        <td>0</td>
        <td>0</td>
        <td>0</td>
        <td>0</td>
        <td>0</td>
    </tr>
    <tr>
        <td>x3</td>
        <td>0</td>
        <td>-1.8</td>
        <td>0.1</td>
        <td>0</td>
        <td>0</td>
        <td>0</td>
        <td>0</td>
        <td>0</td>
        <td>0</td>
    </tr>
  </tbody>
</table>


```matlab
% ---------- Unit-step response of designed system ----------
A = [0 1 0;0 0 1;0 -2 -3];
B = [0;0;1]
C = [1 0 0];
D = [0];
K = [100.0000 53.1200 11.6711];
k1 = K(1); k2 = K(2); k3 = K(3);
% ***** Define the state matrix, control matrix, output matrix,
% and direct transmission matrix of the designed systems as AA,
% BB, CC, and DD *****
AA = A - B*K;
BB = B*k1;
CC = C;
DD = D;
t = 0:0.01:8;
[y,x,t] = step (AA,BB,CC,DD,1,t);
plot(t,x)
grid
title('Response Curves x1, x2, x3, versus t')
xlabel('t Sec')
ylabel('x1,x2,x3')
text(2.6,1.35,'x1')
text(1.2,1.5,'x2')
text(0.6,3.5,'x3')
```

9

# 2. Practical Examples

* **Example 2: (<span style="color: red">exercises in class</span>)**

a) Simulate the output with the following references:

* $r = t^3 - t^2 + t - 1, t = (0, 10) \text{ s}$

* $r = \sin(2*t + \pi/9), t = (0, 10) \text{ s}$

* $r = \sin(2*t + \pi/4) + \cos(2*t), t = (0, 10) \text{ s}$

b) Try to use different matrices **Q** and discuss on the obtained results.

10

# 3. Q-Augmented LQR Controller

**Q-Augmented LQR Controller**

- Include additional state(s) — typically the integral of the error — so that the controller penalizes not just the **original state vector x**, but also **accumulated error**.

- Control error: $\mathbf{e} = \mathbf{r} - \mathbf{y} \Rightarrow \mathbf{x}_i = \int \mathbf{e} \cdot dt = \int (\mathbf{r} - \mathbf{y}) dt$ (integral of control error)
$\Rightarrow \dot{\mathbf{x}}_i = \mathbf{r} - C\mathbf{x}$

- Augmented system: $\begin{aligned} \dot{\mathbf{x}} &= A\mathbf{x} + B\mathbf{u} \\ \dot{\mathbf{x}}_i &= \mathbf{r} - C\mathbf{x} \end{aligned} \Rightarrow \begin{bmatrix} \dot{\mathbf{x}} \\ \dot{\mathbf{x}}_i \end{bmatrix} = \begin{bmatrix} A & 0 \\ -C & 0 \end{bmatrix} \begin{bmatrix} \mathbf{x} \\ \mathbf{x}_i \end{bmatrix} + \begin{bmatrix} B \\ 0 \end{bmatrix} \mathbf{u} + \begin{bmatrix} 0 \\ I \end{bmatrix} \mathbf{r}$

or $\dot{\mathbf{x}}_a = A_a \mathbf{x}_a + B_a \mathbf{u} + B_r \mathbf{r}$ <span style="color: red">(state equation of Q-augmented model)</span>

where $\mathbf{x}_a = \begin{bmatrix} \mathbf{x} \\ \mathbf{x}_i \end{bmatrix}, A_a = \begin{bmatrix} A & 0 \\ -C & 0 \end{bmatrix}, B_a = \begin{bmatrix} B \\ 0 \end{bmatrix}, B_r = \begin{bmatrix} 0 \\ I \end{bmatrix}$

11

# 3. Q-Augmented LQR Controller

**Example 3**: Design an Q-augmented LQR controller for the following system

![Mass-spring-damper system diagram showing a mass m=2 connected to a wall by a spring k=10 and a damper b=0.5, with an input force F(t) and displacement x.](page_6_image_1_v2.jpg)

where
- **$k_s$**: spring stiffness coefficient
- **$k_d$**: damping coefficient
- **$m$**: mass
- **$F(t)$**: system input

12

# 3. Q-Augmented LQR Controller

* **Example 3: (*Solution*)**

![Diagram of a mass-spring-damper system with mass m=2, spring constant k=10, and damping coefficient b=0.5. An external force F(t) is applied, and displacement is x.](page_7_image_2_v2.jpg)

- SS model:
$$ \begin{aligned} \dot{\mathbf{x}} &= A\mathbf{x} + B\mathbf{u} \\ \mathbf{y} &= C\mathbf{x} \end{aligned} $$
where $\mathbf{x} = \begin{bmatrix} x_1 \\ x_2 \end{bmatrix}$, $A = \begin{bmatrix} 0 & 1 \\ -\frac{k_s}{m} & -\frac{k_d}{m} \end{bmatrix}$, $B = \begin{bmatrix} 0 \\ \frac{1}{m} \end{bmatrix}$, $C = \begin{bmatrix} 1 & 0 \end{bmatrix}$

- Q-augmented model:
$$ \begin{aligned} \dot{\mathbf{x}}_a &= A_a\mathbf{x}_a + B_a\mathbf{u} + B_r\mathbf{r} \\ \mathbf{y} &= C\mathbf{x} \end{aligned} $$
where $\mathbf{x}_a = \begin{bmatrix} \mathbf{x} \\ \mathbf{x}_i \end{bmatrix}$, $A_a = \begin{bmatrix} A & 0 \\ -C & 0 \end{bmatrix}$, $B_a = \begin{bmatrix} B \\ 0 \end{bmatrix}$, $B_r = \begin{bmatrix} 0 \\ I \end{bmatrix}$



# 3. Q-Augmented LQR Controller

* **Example 3: (*Solution*)**

- SS model:
$$ \begin{aligned} \dot{\mathbf{x}} &= A\mathbf{x} + B\mathbf{u} \\ \mathbf{y} &= C\mathbf{x} \end{aligned} $$
- Q-augmented model:
$$ \begin{aligned} \dot{\mathbf{x}}_a &= A_a\mathbf{x}_a + B_a\mathbf{u} + B_r\mathbf{r} \\ \mathbf{y} &= C\mathbf{x} \end{aligned} $$

- Input signal: $\mathbf{u} = -K\mathbf{x}_a = -\begin{bmatrix} K_x & K_i \end{bmatrix} \begin{bmatrix} \mathbf{x} \\ \mathbf{x}_i \end{bmatrix} = -K_x\mathbf{x} - K_i\mathbf{x}_i$

![Control system block diagram showing a reference signal r, error e, integral gain Ki, plant B, state feedback Kx, and output y.](page_7_image_1_v2.jpg)

13
14

# 3. Q-Augmented LQR Controller

![decorative square](page_8_image_1_v2.jpg)

Example 3: (*Solution*)

**K =**
**5.3512 10.3897 -1.7321**

```matlab
ks=1
kd=0.1
m=10
A=[0 1; -ks/m -kd/m];
B=[0; 1/m];
C=[1 0]
D=[0]
% compute the open loop poles
openLoopPoles=eig(A)
% form the augmented system
Aa=[A [0; 0]; -C 0]
Ba=[B;0]
Ca=[C, 0]
ssModel=ss(Aa,Ba,Ca,[])
% design the optimal controller
Q=[3 0 0;
0 3 0;
0 0 3;]
R=1;
N=zeros(3,1)
[K,~,~] = lqr(ssModel,Q,R,N)
% check the closed loop poles
eig(Aa-Ba*K)
% extract feedback matrices
Kx=K(:,1:2)
Ki=K(3)
```



# 3. Q-Augmented LQR Controller

![decorative square](page_8_image_2_v2.jpg)

Example 3: (**exercises in class**)

a) Simulate the output with the following references:

* ✓ $r = t^3 - t^2 + t - 1, t = (0, 10) \text{ s}$
* ✓ $r = \sin(2*t + \pi/9), t = (0, 10) \text{ s}$
* ✓ $r = \sin(2*t + \pi/4) + \cos(2*t), t = (0, 10)$
* ✓ $r = \text{waveform with amplity} = 1; \text{frequency} = 0.2, t = (0, 10) \text{ s}$

b) Try to use different matrices **Q** and discuss on the obtained results.

15
16

# 3. Q-Augmented LQR Controller

* **Example 4**: Determine signal gain matrix **K** so that performance index J is minimal

Consider the same inverted pendulum system, where $M = 2\text{ kg}$, $m = 0.1\text{ kg}$, and $l = 0.5\text{ m}$. The system equations are given by

$$
\begin{aligned}
\dot{\mathbf{x}} &= \mathbf{Ax} + \mathbf{Bu} \\
y &= \mathbf{Cx} \\
u &= -\mathbf{Kx} + k_I \xi \\
\dot{\xi} &= r - y = r - \mathbf{Cx}
\end{aligned}
\qquad \mathbf{x} = \begin{bmatrix} x_1 \\ x_2 \\ x_3 \\ x_4 \end{bmatrix} = \begin{bmatrix} \theta \\ \dot{\theta} \\ x \\ \dot{x} \end{bmatrix}
$$

$$
\mathbf{A} = \begin{bmatrix} 0 & 1 & 0 & 0 \\ 20.601 & 0 & 0 & 0 \\ 0 & 0 & 0 & 1 \\ -0.4905 & 0 & 0 & 0 \end{bmatrix}, \quad \mathbf{B} = \begin{bmatrix} 0 \\ -1 \\ 0 \\ 0.5 \end{bmatrix}, \quad \mathbf{C} = \begin{bmatrix} 0 & 0 & 1 & 0 \end{bmatrix}
$$


<table>
  <thead>
    <tr>
        <th>Diagram of Inverted Pendulum System</th>
    </tr>
  </thead>
  <tbody>
    <tr>
        <td> </td>
    </tr>
  </tbody>
</table>

(Note: The visual region ifds depicts a physical diagram of an inverted pendulum on a cart with variables $z, x, \ell \sin \theta, \ell \cos \theta, m, mg, \ell, \theta, P, u, M$ labeled.)



# 3. Q-Augmented LQR Controller

* **Example 4**: Determine signal gain matrix **K**

![Block diagram of the Q-Augmented LQR Controller showing the feedback loop with gain matrix K (components k1 through k4), integral gain kI, and the plant dynamics Ax + Bu and Cx.](page_9_image_1_v2.jpg)

17
18

# 3. Q-Augmented LQR Controller

## Example 4: (*Solution*)

‒ Error equation: $$ \begin{bmatrix} \dot{\mathbf{x}}_e(t) \\ \dot{\xi}_e(t) \end{bmatrix} = \begin{bmatrix} \mathbf{A} & \mathbf{0} \\ -\mathbf{C} & 0 \end{bmatrix} \begin{bmatrix} \mathbf{x}_e(t) \\ \xi_e(t) \end{bmatrix} + \begin{bmatrix} \mathbf{B} \\ 0 \end{bmatrix} u_e(t) \qquad \mathbf{e} = \begin{bmatrix} \mathbf{x}_e \\ \xi_e \end{bmatrix} = \begin{bmatrix} \mathbf{x}(t) - \mathbf{x}(\infty) \\ \xi(t) - \xi(\infty) \end{bmatrix} $$

or $$ \dot{\mathbf{e}} = \hat{\mathbf{A}}\mathbf{e} + \hat{\mathbf{B}}u_e $$

where $$ \hat{\mathbf{A}} = \begin{bmatrix} \mathbf{A} & \mathbf{0} \\ -\mathbf{C} & 0 \end{bmatrix} = \begin{bmatrix} 0 & 1 & 0 & 0 & 0 \\ 20.601 & 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 1 & 0 \\ -0.4905 & 0 & 0 & 0 & 0 \\ 0 & 0 & -1 & 0 & 0 \end{bmatrix}, \quad \hat{\mathbf{B}} = \begin{bmatrix} \mathbf{B} \\ 0 \end{bmatrix} = \begin{bmatrix} 0 \\ -1 \\ 0 \\ 0.5 \\ 0 \end{bmatrix} $$



# 3. Q-Augmented LQR Controller

## Example 4: (*Solution*)

‒ Control signal: $u_e = -\hat{\mathbf{K}}\mathbf{e}$

where $$ \hat{\mathbf{K}} = \begin{bmatrix} \mathbf{K} & \vdots & -k_I \end{bmatrix} = \begin{bmatrix} k_1 & k_2 & k_3 & k_4 & \vdots & -k_I \end{bmatrix} $$
$$ \mathbf{e} = \begin{bmatrix} \mathbf{x}_e \\ \xi_e \end{bmatrix} = \begin{bmatrix} \mathbf{x}(t) - \mathbf{x}(\infty) \\ \xi(t) - \xi(\infty) \end{bmatrix} $$

‒ Performance index: $$ J = \int_{0}^{\infty} (\mathbf{e}^* \mathbf{Q} \mathbf{e} + u^* R u) \, dt \quad \text{with} \quad \mathbf{Q} = \begin{bmatrix} 100 & 0 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 & 0 \\ 0 & 0 & 1 & 0 & 0 \\ 0 & 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 0 & 1 \end{bmatrix}, \quad R = 0.01 $$

$$ k_1 = -188.0799, \quad k_2 = -37.0738, \quad k_3 = -26.6767, \quad k_4 = -30.5824, \quad k_I = -10.0000 $$

19
20

# 3. Q-Augmented LQR Controller

* **Example 4: (*Solution*)**

```matlab
% Design of quadratic optimal control system
A = [0 1 0 0;20.601 0 0 0;0 0 0 1;-0.4905 0 0 0];
B = [0;-1;0;0.5];
C = [0 0 1 0];
D = [0];
Ahat = [A zeros(4,1);-C 0];
Bhat = [B;0];
Q = [100 0 0 0 0;0 1 0 0 0;0 0 1 0 0;0 0 0 1 0;0 0 0 0 1];
R = [0.01];
Khat = lqr(Ahat,Bhat,Q,R)
Khat =
-188.0799 -37.0738 -26.6767 -30.5824 10.0000
```



# 3. Q-Augmented LQR Controller

* **Example 4: Unit-Step Response**

‒ System equation:
$$ \begin{bmatrix} \dot{\mathbf{x}} \\ \dot{\xi} \end{bmatrix} = \begin{bmatrix} \mathbf{A} & \mathbf{0} \\ -\mathbf{C} & 0 \end{bmatrix} \begin{bmatrix} \mathbf{x} \\ \xi \end{bmatrix} + \begin{bmatrix} \mathbf{B} \\ 0 \end{bmatrix} u + \begin{bmatrix} \mathbf{0} \\ 1 \end{bmatrix} r $$

$$ \begin{bmatrix} \dot{\mathbf{x}} \\ \dot{\xi} \end{bmatrix} = \begin{bmatrix} \mathbf{A} - \mathbf{BK} & \mathbf{B}k_I \\ -\mathbf{C} & 0 \end{bmatrix} \begin{bmatrix} \mathbf{x} \\ \xi \end{bmatrix} + \begin{bmatrix} \mathbf{0} \\ 1 \end{bmatrix} r $$

‒ Output equation:
$$ y = \begin{bmatrix} \mathbf{C} & 0 \end{bmatrix} \begin{bmatrix} \mathbf{x} \\ \xi \end{bmatrix} + [0]r $$

21
22

# 3. Q-Augmented LQR Controller

* **Example 4: Unit-Step Response**


<table>
  <tbody>
    <tr>
        <td>t (sec)</td>
        <td>x1</td>
    </tr>
    <tr>
        <td>0</td>
        <td>0</td>
    </tr>
    <tr>
        <td>2</td>
        <td>0.018</td>
    </tr>
    <tr>
        <td>4</td>
        <td>-0.004</td>
    </tr>
    <tr>
        <td>6</td>
        <td>0.001</td>
    </tr>
    <tr>
        <td>8</td>
        <td>0</td>
    </tr>
    <tr>
        <td>10</td>
        <td>0</td>
    </tr>
  </tbody>
</table>
<table>
  <tbody>
    <tr>
        <td>t (sec)</td>
        <td>x2</td>
    </tr>
    <tr>
        <td>0</td>
        <td>0</td>
    </tr>
    <tr>
        <td>2</td>
        <td>0.035</td>
    </tr>
    <tr>
        <td>4</td>
        <td>-0.005</td>
    </tr>
    <tr>
        <td>6</td>
        <td>0.002</td>
    </tr>
    <tr>
        <td>8</td>
        <td>0</td>
    </tr>
    <tr>
        <td>10</td>
        <td>0</td>
    </tr>
  </tbody>
</table>
<table>
  <tbody>
    <tr>
        <td>t (sec)</td>
        <td>x3</td>
    </tr>
    <tr>
        <td>0</td>
        <td>0</td>
    </tr>
    <tr>
        <td>2</td>
        <td>0.2</td>
    </tr>
    <tr>
        <td>4</td>
        <td>0.8</td>
    </tr>
    <tr>
        <td>6</td>
        <td>1.0</td>
    </tr>
    <tr>
        <td>8</td>
        <td>1.0</td>
    </tr>
    <tr>
        <td>10</td>
        <td>1.0</td>
    </tr>
  </tbody>
</table>
<table>
  <tbody>
    <tr>
        <td>t (sec)</td>
        <td>x4</td>
    </tr>
    <tr>
        <td>0</td>
        <td>0</td>
    </tr>
    <tr>
        <td>2</td>
        <td>0.35</td>
    </tr>
    <tr>
        <td>4</td>
        <td>0.15</td>
    </tr>
    <tr>
        <td>6</td>
        <td>0.02</td>
    </tr>
    <tr>
        <td>8</td>
        <td>0</td>
    </tr>
    <tr>
        <td>10</td>
        <td>0</td>
    </tr>
  </tbody>
</table>
<table>
  <tbody>
    <tr>
        <td>t (sec)</td>
        <td>x5</td>
    </tr>
    <tr>
        <td>0</td>
        <td>0</td>
    </tr>
    <tr>
        <td>2</td>
        <td>1.8</td>
    </tr>
    <tr>
        <td>4</td>
        <td>2.8</td>
    </tr>
    <tr>
        <td>6</td>
        <td>2.8</td>
    </tr>
    <tr>
        <td>8</td>
        <td>2.8</td>
    </tr>
    <tr>
        <td>10</td>
        <td>2.8</td>
    </tr>
  </tbody>
</table>


```matlab
% Unit-step response
A = [0 1 0 0;20.601 0 0 0;0 0 0 1;-0.4905 0 0 0];
B = [0;-1;0;0.5];
C = [0 0 1 0];
D = [0];
K = [-188.0799 -37.0738 -26.6767 -30.5824];
kI = -10.0000;
AA = [A-B*K B*kI; -C 0];
BB = [0;0;0;0;1];
CC= [C 0];
DD = D;
t = 0:0.01:10;
[y,x,t] = step(AA,BB,CC,DD,1,t);
x1 = [1 0 0 0 0]*x';
x2 = [0 1 0 0 0]*x';
x3 = [0 0 1 0 0]*x';
x4 = [0 0 0 1 0]*x';
x5 = [0 0 0 0 1]*x';
subplot(3,2,1); plot(t,x1); grid;
xlabel('t (sec)'); ylabel('x1')
subplot(3,2,2); plot(t,x2); grid;
xlabel('t (sec)'); ylabel('x2')
subplot(3,2,3); plot(t,x3); grid;
xlabel('t (sec)'); ylabel('x3')
subplot(3,2,4); plot(t,x4); grid;
xlabel('t (sec)'); ylabel('x4')
subplot(3,2,5); plot(t,x5); grid;
xlabel('t (sec)'); ylabel('x5')
```

23

# 3. Q-Augmented LQR Controller

* **Example 4: (exercises in class – Exercise 8)**

a) Simulate the output with the following references:

* $r = t^3 - t^2 + t - 1, t = (0, 10) \text{ s}$

* $r = \sin(2*t + \pi/9), t = (0, 10) \text{ s}$

* $r = \text{waveform with amplity} = 1; \text{ frequency} = 0.2, t = (0, 10) \text{ s}$

b) Try to use different matrices **Q** and discuss on the obtained results.

24

# 4. Selection of Weighting Matrices

**Problem statement:**

- LQR performance index: $$ J = \int_{0}^{\infty} (\mathbf{x}^* \mathbf{Q} \mathbf{x} + \mathbf{u}^* \mathbf{R} \mathbf{u}) \, dt $$

- An LQR solution which is optimal with one choice of **Q** and **R** will not normally be optimal for other choices of the **Q** and **R** matrices.

**Solution:**

- For SISO, one can fix the matrix **R** to unity and adjust only the matrix **Q**.

$$ \begin{aligned} \Rightarrow \frac{J}{R} &= \int_{0}^{\infty} \left[ \mathbf{x}^{\text{T}}(t) \frac{\mathbf{Q}}{R} \mathbf{x}(t) + \mathbf{u}^{\text{T}}(t) \mathbf{u}(t) \right] dt \\ &= \int_{0}^{\infty} \left[ \mathbf{x}^{\text{T}}(t) \mathbf{Q}_1 \mathbf{x}(t) + u^2(t) \right] dt, \quad \text{where} \quad \mathbf{Q}_1 = \mathbf{Q} R^{-1} \end{aligned} $$



# 4. Selection of Weighting Matrices

**Cheap control:**

- Use any large control signals to ensure the dynamic behavior of the system.

For $R = 1$, the weight on $\mathbf{x}(t)$, i.e., $\mathbf{Q}$, should be very large.

- Use of single tuning knob $\rho$: $\widehat{\mathbf{Q}} = \rho \mathbf{Q}$

**Expensive control:**

- With an expensive control strategy, the control cost is assumed to be quite large.

For $R = 1$, the weight on $\mathbf{x}(t)$, i.e., $\mathbf{Q}$, should be very small.

25
26

27

# 4. Selection of Weighting Matrices

**Example 5:**

‒ Consider a plant model:

$$ \dot{\boldsymbol{x}}(t) = \begin{bmatrix} -0.2 & 0.5 & 0 & 0 & 0 \\ 0 & -0.5 & 1.6 & 0 & 0 \\ 0 & 0 & -14.3 & 85.8 & 0 \\ 0 & 0 & 0 & -33.3 & 100 \\ 0 & 0 & 0 & 0 & -10 \end{bmatrix} \boldsymbol{x}(t) + \begin{bmatrix} 0 \\ 0 \\ 0 \\ 0 \\ 30 \end{bmatrix} u(t), \quad y = [1, 0, 0, 0, 0]\boldsymbol{x} $$

Assume that **Q** = diag([10,20,6,2,5]), plot the step responses of the system with different $\rho = (1, 10, 100, 1000)$.



# 4. Selection of Weighting Matrices

**Example 5: (solution)**

```matlab
A=[-0.2,0.5,0,0,0;
    0,-0.5,1.6,0,0;
    0,0,-14.3,85.8,0;
    0,0,0,-33.3,100;
    0,0,0,0,-10];
B=[0; 0; 0; 0; 30];
Q=diag([10,20,6,2,5]);
R=1;
C=[1,0,0,0,0];
D=0;
for rho=[1,10,100,10000]
    [K,P]=lqr(A,B,rho*Q,R);
    Ac=A-B*K;
    step(ss(Ac,B,C,D)); hold on
end
```


Step Response
<table>
  <thead>
    <tr>
        <th>Time (seconds)</th>
        <th>ρ = 1</th>
        <th>ρ = 10</th>
        <th>large value of ρ</th>
    </tr>
  </thead>
  <tbody>
    <tr>
        <td>0</td>
        <td>0</td>
        <td>0</td>
        <td>0</td>
    </tr>
    <tr>
        <td>5</td>
        <td>0.22</td>
        <td>0.07</td>
        <td>0.02</td>
    </tr>
    <tr>
        <td>10</td>
        <td>0.26</td>
        <td>0.08</td>
        <td>0.02</td>
    </tr>
    <tr>
        <td>15</td>
        <td>0.27</td>
        <td>0.08</td>
        <td>0.02</td>
    </tr>
    <tr>
        <td>20</td>
        <td>0.27</td>
        <td>0.08</td>
        <td>0.02</td>
    </tr>
  </tbody>
</table>


When $\rho$ increases, the magnitudes of the state responses are significantly reduced, which in turn makes the output signal smaller.

28