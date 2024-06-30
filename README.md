# RV-VP2: RISC-V based Virtual Prototype (VP) with RV32 P-extensions Support

# Usage
The "RV-VP2" platform is implemented based on open source RISC-V based Vitual Prototype (RV-VP) reference [1].

[1] V. Herdt, D. Große, P. Pieper, and R. Drechsler,RISC-V based virtual prototype: An extensible and configurable platform for the system-level, In Journal of Systems Architecture, 2020, (url: https://github.com/agra-uni-bremen/riscv-vp)

#### 1) Build requirements

Mainly the usual build tools and boost is required:

On Ubuntu 20, install these:
```bash
sudo apt-get install autoconf automake autotools-dev curl libmpc-dev libmpfr-dev libgmp-dev gawk build-essential bison flex texinfo libgoogle-perftools-dev libtool patchutils bc zlib1g-dev libexpat-dev libboost-iostreams-dev libboost-program-options-dev libboost-log-dev qt5-default
```

#### 2) Build this RISC-V Virtual Prototype:


Check out all submodules (`git submodule update --init --recursive`), and type `make all`. This script does the following for you:

>
>i) in *vp/dependencies* folder (will download and compile SystemC, and build a local version of the softfloat library):
>
>```bash
>./build_systemc_233.sh
>./build_softfloat.sh
>```
>
>
>ii) in *vp* folder (requires the *boost* C++ library):
> 
>```bash
>mkdir build
>cd build
>cmake ..
>make install
>```

#### 3) Building SW examples using the GNU toolchain

##### Requirements

In order to test the software examples, a configured RISC-V GNU toolchain with P-extension is required in your `$PATH`.
Several standard packages are required to build the toolchain.
For more information on prerequisites for the RISC-V GNU toolchain visit https://github.com/riscv/riscv-gnu-toolchain.
With the packages installed, the toolchain can be build as follows:

```bash
# in some source folder
git clone https://github.com/riscv/riscv-gnu-toolchain.git
cd riscv-gnu-toolchain
git clone https://github.com/plctlab/riscv-gcc -b riscv-gcc-p-ext riscv-gcc-p-ext
git clone https://github.com/plctlab/riscv-binutils-gdb -b riscv-binutils-p-ext riscv-binutils-p-ext
./configure --prefix=$RISCV --with-arch=rv32imap_zifencei --with-abi=ilp32 --with-gcc-src=pwd/riscv-gcc-p-ext --with-binutils-src=pwd/riscv-binutils-p-ext
In makefile set INSTALL_DIR := /opt/riscv
sudo make
```

##### Running the examples

In *sw*:

```bash
cd p-ext-add8	    # can be replaced with different example
make                # (requires RISC-V GNU toolchain in PATH)
make sim            # (requires *riscv-vp*, i.e. *vp/build/bin/riscv-vp*, executable in PATH)
```

Please note, if *make* is called without the *install* argument in step 2, then the *riscv-vp* executable is available in *vp/build/src/platform/basic/riscv-vp*.

This will also copy the VP binaries into the *vp/build/bin* folder.

# Citation
If you use this code as part of your work, please cite the following paper:

M. Ali and E. Aliagha, M. Elnashar and D. Göhringer, RV-VP2: Unlocking the potential of RISC-V Packed-SIMD for Embedded Processing, In International Conference on Embedded Computer Systems: Architectures, Modeling and Simulation (SAMOS), July. 2024

# Contact Info
M.Sc. Muhammad Ali, Technische Universität Dresden, muhammad.ali@tu-dresden.de,

Google Scholar: https://scholar.google.com/citations?hl=en&user=lQDppZ8AAAAJ

# Acknowledgment
This work has been partially funded by the German Federal Ministry of Education and Research BMBF as part of the DAKORE (Datenfunknetz mit Adaptivhardware und KI-Optimierung zur Reduktion des Energieverbrauchs) project under grant agreement number 16ME0433K and by the German Research Foundation (“Deutsche Forschungsgemeinschaft”) (DFG) under Project-ID 287022738 TRR 196 for Project S05.