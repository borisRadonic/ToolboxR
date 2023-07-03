# ToolboxR Control Library

This library contains the basic time discrete blocks commonly used in discrete time modeling and Fuzzy Control blocks that provides a comprehensive set of features and functionalities for developing and implementing fuzzy logic-based control systems.

The Discrete Time part of ToolboxR Library in C++ provides (should provide) the following features and functionalities:

    Abstraction of discrete time modeling: The library provides a set of classes and functions that allow users to define and work with discrete time models easily. It sabstracts away the low-level details of managing discrete time steps and state transitions.

    Model representation: The library offers offer a way to represent the state variables, inputs, outputs, and parameters of a discrete time model. This is achieved through classes or data structures that encapsulate these elements.

    Model initialization and configuration: The library provides mechanisms to initialize and configure a discrete time model. This includes setting the initial state values, specifying input values, and assigning parameter values.

    Step-wise simulation: The library offers functions or methods to simulate the behavior of the model over discrete time steps. Users are able to advance the model state from one time step to another, updating the state variables based on the defined model equations or rules.

    Support for different model types: The library accommodates various types of discrete time models, such as difference equations, discrete time integral, state-space models and some other relevant models. It is flexible enough to handle different modeling paradigms and approaches.

    Input and output handling: The library should provide mechanisms to handle inputs and outputs of the model. This involves reading input values from external sources, providing inputs to the model, and collecting and processing output values generated by the model.

    Flexibility and extensibility: The library has been designed to be flexible and extensible, allowing users to customize and extend its functionality according to their specific needs. It provides interfaces for users to incorporate their own model equations and algorithms.

    Documentation and examples: The library should be well-documented, with clear explanations and examples of how to use its features and functionalities. This will help users understand and leverage the library effectively.

By providing these features and functionalities, a Discrete Time Model Library in C++ can facilitate the development and simulation of discrete time models in a convenient and efficient manner.

Here are some key aspects of the ToolboxR Fuzzy Control Blocks:

    Fuzzy set representation: The library provides classes or data structures to define and represent fuzzy sets and their associated membership functions.
    			      It supports the following types of membership functions: Singleton, Gaussian,Bell Shaped, Sigmoidal, Triangular, Trapezoidal, PI-shaped, S-shaped, Z-Shaped,
                              Trapezoidal, Fuzzy-Small function, Fuzzy Large function  and Linear Sugeno membership function.

    Fuzzy rule-based systems: The library supports the construction and execution of fuzzy rule-based systems. It should provide a way to define linguistic variables, fuzzy rules, and the corresponding fuzzy inference mechanism for mapping inputs to outputs.

    Membership function operations: The library should offer a range of operations on fuzzy sets and membership functions, such as fuzzy set intersection, union, complement, scaling, and aggregation. These operations are essential for fuzzy logic computations and reasoning.

    Fuzzy inference methods: The library should support various fuzzy inference methods, including Mamdani and Sugeno. It should provides functions and classes to perform fuzzy inference and compute the output values based on the input variables and fuzzy rule base.

    Defuzzification methods: The library should include a variety of defuzzification methods to convert the fuzzy output into a crisp value or action.
   			     Common defuzzification methods include centroid, weighted average, and maximum membership.

    Fuzzy control system design tools: The library should provide tools or utilities to assist in the design and tuning of fuzzy control systems. This may include functions for rule base optimization, rule extraction from data, or parameter tuning algorithms.

    Fuzzy logic operators: The library supports a range of fuzzy logic operators, such as fuzzy AND, fuzzy OR, fuzzy NOT, and some fuzzy implication operators. These operators are fundamental for fuzzy rule evaluation and inference.

    Import/Export: This library provides basic Import/Export utility classes which enable Import of MATLAB/Simulink Fuzzy Models (FIS File Format)

By providing these features and functionalities, a ToolboxR could empower developers to easily implement and deploy fuzzy logic-based control systems for various applications, such as robotics, automation, process control, and decision support systems.


A discrete time model is a mathematical representation of a system that evolves over time in discrete steps or intervals. 
In contrast to continuous time models, which describe systems that change continuously over time, discrete time models capture the behavior of a system at specific points or moments in time.

In a discrete time model, time is represented as a sequence of discrete time points or steps, typically denoted by integers such as 0, 1, 2, and so on. At each time step, the state of the system is updated based on the previous state and any external inputs or influences.

The dynamics of a discrete time model are often described by a set of equations or rules that govern how the system transitions from one state to another. These equations or rules define how the system variables change over time and can incorporate factors such as delays,
feedback, and discrete events.

Discrete time models are widely used in various fields, including mathematics, physics, engineering, computer science, economics, and biology. They are particularly useful when the underlying system being modeled naturally exhibits discrete behaviors or when the measurements
or observations of the system are made at discrete intervals. Discrete time models can be analyzed using mathematical techniques such as recurrence relations, difference equations, and discrete dynamical systems theory.

Overall, discrete time models provide a valuable framework for understanding and predicting the behavior of systems that evolve in discrete steps over time.

The main classes are:

Discrete Time Delay

	The Delay class holds and delays its input by the sample period you specify.

Difference

	The Difference class outputs the current input value minus the previous input value.

Te Discrete Time Derivative

	The Discrete Derivative pproximates the derivative of its input.

Discrete FIR Filter
	
	Discrete FIR Filter class implements a discrete-time finite impulse response (FIR) filter.


Discrete First-Order IIR Filter
	The IIR Filter class implements a discrete first-order infinite impulse response (IIR) filter on the specified input signal.

Discrete Second-Order IIR Filter
	The IIR Filter class implements a discrete second-order infinite impulse response (IIR) filter on the specified input signal.	

Discrete IIR Filter
	The IIR Filter class implements a discrete-time infinite impulse response (IIR) filter on the specified input signal.


Discrete Integrator
	Discrete Integrator class performs discrete-time integration or accumulation of signal. It implements Trapezoidal, Forward Euler and Backward Euler integration methods.

Discrete PID Controller
	The Discrete PID Controller class implements a PID controller (PID, PI, PD only or I only). It includes Output saturation limits and anti-windup mechanism.

Discrete State-Space

 The Discrete State-Space class represents the discrete-time dynamic system in state-space form. A, B, C, and D matrices and intial conditions can be specified. It uses Eigen Library.
	
WaveFormTracer
	This utility classs can be used to export selected signals in gnuplot format file. (Not implemented at the moment!)



## Project Status

* Work in progress...
