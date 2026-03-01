# VFH Ultrasonic Sensor — Arduino + Python GUI

Real-time **Vector Field Histogram (VFH)** obstacle detection using a single **HC-SR04** ultrasonic sensor, Arduino Uno, and a Python GUI with live polar histogram visualization.


https://github.com/user-attachments/assets/1b9d81cc-97cc-47d6-8323-0f53fee3ab72


---

## 📸 Preview

| FOV Sensor View | Polar Obstacle Density | Distance History |
|:-:|:-:|:-:|
| Cone ±15°, obstacle point | Radial bars per sector | Real-time trend |

---

## 🔧 Hardware

### Wiring

| HC-SR04 Pin | Arduino Uno |
|:-----------:|:-----------:|
| VCC         | 5V          |
| GND         | GND         |
| TRIG        | D2          |
| ECHO        | D3          |
---

### 1. Distance — Sensor Reading

The distance $d_{ij}$ is read directly from the HC-SR04 ultrasonic sensor via Arduino serial output:

$$d_{ij} = \frac{t_{echo} \times 0.0343}{2} \quad \text{(cm)}$$

where $t_{echo}$ is the echo pulse duration in microseconds.

---

### 2. Obstacle Vector Magnitude

Each cell's contribution to the histogram is weighted by its distance:

$$m_{ij} = c_{ij}^{2} \cdot (a - b \cdot d_{ij})$$

| Symbol | Value | Description |
|:------:|:-----:|:------------|
| $c_{ij}$ | 1.0 | Certainty value of the cell |
| $a$ | 1.0 | Positive constant |
| $b$ | $1/200$ | Positive constant ($= a / d_{max}$) |
| $d_{max}$ | 200 cm | Maximum relevant distance |

> $m_{ij} = 0$ when $d_{ij} \geq a/b$

---

### 3. Sector Index

Each bearing angle $\beta$ is mapped to a histogram sector $k$:

$$k = \left\lfloor \frac{\beta}{\alpha} \right\rfloor$$

| Symbol | Value | Description |
|:------:|:-----:|:------------|
| $\alpha$ | 10° | Angular resolution per sector |
| $\beta$ | 0° | Bearing of the sensor (fixed, front) |
| $N$ | 36 | Total number of sectors ($= 360 / \alpha$) |

Because the HC-SR04 has a **±15° FOV**, the magnitude is distributed across neighbouring sectors with a distance-weighted factor:

$$w_{offset} = 1 - \frac{|offset|}{FOV/2 + 1}$$

---

### 4. Polar Obstacle Density (Raw Histogram)

The raw histogram value for sector $k$ is the sum of all obstacle magnitudes mapping to that sector:

$$h_k = \sum_{(i,j) \in S_k} m_{ij}$$

where $S_k$ is the set of cells whose bearing falls within sector $k$.

---

### 5. Smoothed Histogram

A moving average filter of window size $2l+1$ is applied to reduce noise:

$$h'_k = \frac{1}{2l+1} \sum_{i=-l}^{l} h_{k+i}$$

| Symbol | Value | Description |
|:------:|:-----:|:------------|
| $l$ | 2 | Smoothing half-window |
| $2l+1$ | 5 | Total window size |

---

### 6. Threshold Classification

Each sector is classified as **blocked**, **free**, or **uncertain**:

$$H^b_k = \begin{cases} 1 & \text{if } h'_k > T_{high} \quad \Rightarrow \textbf{Blocked} \\ 0 & \text{if } h'_k < T_{low} \quad \Rightarrow \textbf{Free} \\ h'_k & \text{otherwise} \quad \Rightarrow \textbf{Uncertain} \end{cases}$$

| Symbol | Value | Description |
|:------:|:-----:|:------------|
| $T_{high}$ | 0.50 | Upper threshold — blocked |
| $T_{low}$  | 0.15 | Lower threshold — free |

---

### 7. Steering Direction

The robot steers toward the **nearest free sector** to the target direction (0° = straight ahead):

$$\theta_{steer} = \arg\min_{\theta \in \Theta_{free}} \left| \theta - \theta_{target} \right|$$

where $\Theta_{free}$ is the set of angles corresponding to free sectors, and $\theta_{target} = 0°$.

> When **all sectors are free** (no obstacle), $\theta_{steer}$ returns to **0°** (target direction).

---

### 8. Angular Velocity

$$\omega = k_\omega \cdot \theta_{steer}$$

| Symbol | Value | Description |
|:------:|:-----:|:------------|
| $k_\omega$ | 2.5 | Angular velocity gain |
| $\theta_{steer}$ | rad | Steering angle |

---

### 9. Linear Velocity

$$V = V_{max} \cdot \left(1 - \frac{|\theta_{steer}|}{\theta_{max}}\right)$$

| Symbol | Value | Description |
|:------:|:-----:|:------------|
| $V_{max}$ | 100 cm/s | Maximum linear velocity |
| $\theta_{max}$ | 180° | Maximum steering angle |

> $V$ decreases as the steering angle increases. $V = 0$ when fully blocked.

---

## 🖥️ GUI Panels

| Panel | Description |
|-------|-------------|
| **FOV Sensor View** | Top-down cone view (±15°) with live obstacle point and distance label |
| **Polar Obstacle Density** | Radial bar per sector — 🔴 Blocked / 🟢 Free / 🟠 Uncertain. Arrows show target (yellow) and steering (purple) directions |
| **Distance History** | Rolling 100-sample trend with danger zones (red < 30 cm, orange < 80 cm) |

---

---

## 📚 Reference

> Borenstein, J. & Koren, Y. (1991). *The Vector Field Histogram — Fast Obstacle Avoidance for Mobile Robots*. IEEE Transactions on Robotics and Automation, 7(3), 278–288.
