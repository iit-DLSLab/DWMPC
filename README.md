# DWMPC: Distributed Whole-Body Model Predictive Control


| Simulation | Real Robot |
| -------- | ------- |
|  <img src="https://github.com/user-attachments/assets/a56335b6-bb96-4cc3-919e-2314afa77141" width=500>  | <img src="https://github.com/user-attachments/assets/438273d1-2ef8-47f5-89f8-bd6dbaba28c7" width=500> |

<div align="center">
  2024 IEEE/RSJ International Conference on Intelligent Robots and Systems
</div>

<div align="center">
  <a href="#Installation"><b>Installation</b></a> |
  <a href="https://arxiv.org/abs/2403.11742v3"><b>PrePrint</b></a> |
  <a href=https://www.youtube.com/watch?v=Yar4W-Vlh2A><b>Video</b></a>|
  <a href=https://sites.google.com/view/dwmpc/home><b>WebSite</b></a>
  
</div>

DWMPC is a library for distributed model predictive control (MPC) of quadruped robots. This repository includes the core MPC controller in cpp and example usage with the `gym-quadruped` environment in python.


## Dependencies
Before proceeding with the installation, ensure that the following dependencies are available on your system:

- CMake
- GCC
- Python 3 (along with development headers)
- Eigen3
- YAML-CPP
- Pybind11
- ndcurves
- acados

## Installation 

### 1. Clone the Repository
To get started, clone the repository and initialize all submodules:
```bash
git clone https://github.com/iit-DLSLab/DWMPC.git
cd DWMPC
git submodule update --init --recursive
```
### 2. Install System Dependencies
```bash
sudo apt-get install -y cmake g++ python3 python3-dev python3-pip libeigen3-dev libyaml-cpp-dev pybind11-dev
```
Follow the instructions to install the `ndcurves` library from the [official repository](https://github.com/loco-3d/ndcurves)

### 3. Build Acados
Navigate to the `acados` directory and build the library:
```bash
cd third_party/acados
mkdir build && cd build
cmake ..
make install -j4
```
After building, add `acados` to your `LD_LIBRARY_PATH`:
```bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```
### 4. Build DWMPC
From the main `DWMPC` repository, create a build directory and compile the project:
```bash
cd ../../
mkdir build && cd build
cmake ..
make -j8 && sudo make install
```
Add the `DWMPC` library to your environment:
```bash
export LD_LIBRARY_PATH=/usr/lib/dls2/controllers/dwmpc:$LD_LIBRARY_PATH
export PYTHONPATH=$PYTHONPATH:/usr/lib/dls2/controllers/dwmpc
```
### 5. To run the example
For running the example scripts, install the `gym-quadruped` environment from the the [official repository](https://github.com/iit-DLSLab/gym-quadruped)

## Citing this work

```bibtex
@INPROCEEDINGS{amatucciIROS2024,
      title={Accelerating Model Predictive Control for Legged Robots through Distributed Optimization}, 
      author={Lorenzo Amatucci and Giulio Turrisi and Angelo Bratta and Victor Barasuol and Claudio Semini},
       booktitle={2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year={2024}
}
```
