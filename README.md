# CG1 - Exercise 6: Skeletal Animation & Cloth Simulation

This repository demonstrates **skeletal animation** with an elephant model and **cloth simulation** (pendulum chains and curtains) using a spring system. The GUI allows interactive parameter tuning.

---

## Features

### Skeletal Animation

* Loads an elephant mesh (`elephant.obj`) with **bones and weights**.
* Supports multiple animations:

  * Swing Dance
  * Jump
  * Swimming
* Play/pause animations.
* Show/hide skeleton and elephant mesh.

---

### Cloth Simulation

* **Pendulum chain**: Single or multiple pendulums connected by springs.
* **Curtain mesh**: Structural, shear, and flex springs between vertices.
* Adjustable parameters:

  * Spring subdivision
  * Simulation speed
  * Mass, stiffness, drag
  * Integration method: Euler / Trapezoid
* Pin and reset functionality.


**Example: Elepahnt animation and Curtain**

![Curtain Simulation](./assets/curtain.gif)

---
## **🛠️ Building the Project**

The project uses **CMake** for building. Ensure you have **CMake version**  installed and a C++ compiler capable of compiling C++20 (e.g., GCC , Clang , or MSVC ).

### **Steps to Build and Run**

Run these commands in the root directory of the project:

**1\. Configure the project:**

cmake . \-B build \-DCMAKE\_BUILD\_TYPE=Release

This command creates a build directory and configures the project files.

**2\. Build the executable:**

cmake \--build build \--parallel \--config Release

This builds the project executables, placing them inside the build directory.

3\. Run the application:  
To Run the program:
./src/main  
\# or (example for Windows)  
.\\src\\main.exe

***Note: The exact path to the executable may vary based on your CMake setup.***

---

