## 🔍 Project Overview

This project implements a robust **IMU fault detection framework** that combines an **Extended Kalman Filter (EKF)** for bias estimation with a **CUSUM detector** for identifying abnormal sensor behavior. The goal is to reliably detect faults in all six IMU channels (Ax, Ay, Az, p, q, r) while maintaining filter stability and minimizing false alarms.

IMU faults—such as bias drifts, spikes, and sensor degradation—can significantly degrade state estimation in aerospace, robotics, and autonomous systems. This framework provides an end-to-end solution for **early, accurate, and stable fault detection**.

---

## 🧠 What This Project Does

### **1. EKF-Based State & Bias Estimation**
The EKF estimates both system states and IMU bias terms using a nonlinear motion model and GPS observations. Bias states are modeled as random walks, enabling the system to distinguish between normal noise and true sensor faults.

### **2. CUSUM Fault Detection on EKF Innovations**
Fault detection is performed on the EKF innovation signals using positive and negative CUSUM tests. A fault is declared when CUSUM statistics cross a threshold, indicating a significant deviation from expected sensor behavior.

Each IMU channel has two key parameters:
- **γ (leakage)**  
- **δ (threshold)**  

Choosing these parameters correctly is critical for balancing sensitivity and false alarms.

---

## 🎯 3. Baseline Manual Tuning
A manually tuned CUSUM detector was first implemented to establish a baseline.  
Although several faults were detected, manual tuning resulted in:
- Excessive false alarms in some channels  
- Missed detections in others  
- High sensitivity to noise  
This motivated the need for a more systematic optimization approach.

---

## 🚀 4. Multi-Objective Optimization

Three optimization strategies were implemented to tune γ, δ, and σ_bias for each IMU channel:

### **✔ Weighted Sum Method**
Combines detection delay (D), false alarms (F), and bias variance (V) into a single cost function.  
Weights prioritize EKF stability while maintaining fast detection.

### **✔ Pareto-Based Optimization**
Explores trade-offs between competing objectives and selects parameters from the Pareto front using a normalized scoring metric.

### **✔ ε-Constraint Method**
Minimizes detection delay while enforcing strict upper bounds on false alarms and bias variance.  
This method produced several configurations with **zero false alarms**.

---

## 📊 5. Final Validation
The optimized parameters were tested on separate datasets and consistently outperformed the manually tuned baseline:

- Faster detection times  
- Significantly fewer false alarms  
- Stable EKF bias estimation  
- Robust performance across all IMU channels  

---

## ✅ Summary

This project delivers a full **fault detection and optimization framework** suitable for UAVs, aircraft, and robotic platforms.  
By combining EKF estimation, CUSUM detection, and multi-objective optimization, the system achieves:

- Reliable and early fault detection  
- Reduced false alarms  
- Improved estimator stability  

This repository includes:
- MATLAB implementation of EKF + CUSUM  
- Multi-objective optimization routines  
- Visualization scripts  
- Results for weighted, Pareto, and ε-constraint methods  

## ⚙️ Execution Flow

flowchart TD
    A[Start] --> B[Main_weighted.m / Main_Pareto.m / Main_epsilon.m]

    B --> C[Generate Sobol samples for γ, δ, σ_bias]
    C --> D[Loop over channels]
    D --> E[evaluateDesign\_channel / evaluateDesign\_channel\_pareto]

    E --> F[Task3_2Run\_v2.m (run EKF on faulty data)]
    F --> G[runCUSUM\_channel.m (CUSUM on IMU/EKF innovation)]
    G --> H[Compute D (delay), F (false alarms), V (bias variance), J (cost)]
    H --> I[Select best γ, δ, σ\_bias for each of 6 IMU channels]
    I --> J[Save best\_per\_channel\_*.mat]

    J --> K[SID.m]
    K --> L[Load best parameters + nominal stats (dataTask2)]
    L --> M[Run EKF on test/faulty data (dataTask3)]
    M --> N[Channel-wise CUSUM with tuned γ, δ]
    N --> O[Outputs: x\_est, b\_est, Ax\_f…r\_f + plots]
