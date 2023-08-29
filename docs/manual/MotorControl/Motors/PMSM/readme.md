# PMSM & FOC📖

## Introduction to PMSM: A Modern Marvel in Motor Technology

The PMSM represents an advanced tri-phase motor equipped with robust permanent magnets embedded within the rotor. This unique construction results in a marked increase in power density compared to other available motor types.

Historically, the acclaimed AC Induction Motor, a testament to Nikola Tesla's genius from over a century ago, maintained a consistent synchronization with the magnetic field's frequency. However, when juxtaposed with the PMSM, the latter displays a series of distinct advantages. These include:

- Elevated power density
- Compact form factor
- Lighter weight
- Minimal maintenance needs

## Field-Oriented Motor Control (FOC): Bridging the Gap Between DC and PMS Motors

In-depth electrical analysis of the Direct Current (DC) Motors highlights the capability to control the excitation of both stator and rotor independently. This independent control mechanism ensures that torque generation and magnetic flux can be distinctly adjusted.

While PMSMs operate on different foundational principles, they share a crucial feature with DC motors: only the stator currents are subject to control. In PMSMs, the interdependence between flux and torque stems from the utilization of permanent magnets for rotor excitation.

Enter FOC: a sophisticated control methodology aiming to replicate the DC motor's distinct control over torque and flux. Achieving this requires the orthogonality between the rotor and stator magnetic fields.

Employing mathematical transformations, particularly those of Park and Clarke, FOC succeeds in decoupling torque from flux. It's vital to understand that for effective FOC implementation, precise rotor positioning and adept control over stator winding currents are non-negotiable.

## Advantages of FOC: Simplifying the Complex

FOC provides a significant leap in motor control by translating the three-phase abc-reference frame into the dq-coordinate system. This transformation paves the way for an almost homogenous magnetic field across both axes. The result? A system that facilitates independent control over currents in both dq-directions, akin to motors maintaining a speed, Ω, synchronized with the magnetic field induced by stator windings.

## Why the Reduction from 3D to 2D Matters: A Deep Dive

**Physical Insight:** The inherent redundancy in a three-phase motor means the currents across its phases (a, b, c) are interdependent. This interdependence ensures that acquiring values from two phases instantly reveals the third, hence diminishing the need for a third dimension.

**Mathematical Paradigm:** Transitioning from abc to dq coordinates echoes a shift in mathematical coordinates, akin to transitioning from Cartesian to polar coordinates. This change aligns our perspective with the rotor's magnetic field, offering a simpler, quasi-stationary representation for controllers.

**Control Benefits:** Dimension reduction is not just mathematical elegance; it's about ease of control. By shrinking the dimensions from 3 to 2, motor controllers can separately manage the torque and flux components, making the overall control more intuitive and efficient.

**In Conclusion:** As we venture further into the realm of advanced motor technologies, adopting methodologies like FOC for PMSMs becomes not just beneficial but essential. By understanding the core principles and advantages, we can harness the full potential of these motors, driving efficiency and performance to unprecedented heights.

# Mathematical Model of PMSM in dq space after linearization

## 1. Current Dynamics in d-axis

The rate of change of d-axis current $\dot{i_d}$ is determined by:

$$
 \dot{i_d} = \frac{1}{L_d} \left( u_d - R i_{d} + \text{P} \cdot \omega_{M} \cdot L_q \cdot i_{q1} \right) 
$$

Where:
- $L_d$ is the d-axis inductance.
- $u_d$ is the d-axis voltage input.
- $R$ is the stator resistance.
- $i_{d}$ is the d-axis current at the previous time step.
- $omega_{M}$ is the rotor speed.
- $L_q$ is the q-axis inductance.
- $i_{q}$ is the q-axis current at the previous time step.
- ${P}$ is the number of pole pairs in the motor.

## 2. Current Dynamics in q-axis

The rate of change of q-axis current $\dot{i_q}$ is computed by:
$$
\\dot{i_q} = \frac{1}{L_q} \left( u_q - R i_{q} - \omega_{M} \left( \text{P} \cdot L_d \cdot i_{d} - K_{emf} \right) \right) \
$$
Where:
- $L_q$ and $u_q$ are the q-axis inductance and voltage input respectively.
- $K_{emf}$ is the back EMF constant.

## 3. Mechanical Dynamics

The rotor's angular acceleration (\( \alpha_{M} \)) is expressed by:

\[ \alpha_{M} = \frac{1}{J} \left( K_{tq} \cdot i_q - B \cdot \omega_{M1} - T_f - T_{load} \right) \]

Where:
- \( J \) is the moment of inertia.
- \( K_{tq} \) is the torque constant.
- \( B \) is the friction coefficient.
- \( T_f \) is the static friction torque.
- \( T_{load} \) is the load torque.

The rotor speed (\( \omega_{M} \)) and rotor angle (\( \theta_{M} \)) are then obtained by integrating the angular acceleration.

## 4. Output Torque

The electromagnetic torque (\( T_e \)) produced by the motor is calculated as:

\[ T_e = K_{tq} \cdot i_q \]

In this mathematical model:
- The dynamics of d-axis and q-axis currents are influenced by their respective voltages, resistances, inductances, and rotor speed.
- The rotor's mechanical dynamics are determined by the motor's physical properties and the q-axis current, which generates electromagnetic torque.
- The output torque is directly proportional to the q-axis current, indicating the PMSM's ability to generate torque efficiently.