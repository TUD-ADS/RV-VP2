# Virtual Breadboard Protocol

This is the repo containing only the protocol used in https://github.com/agra-uni-bremen/virtual-breadboard and https://github.com/agra-uni-bremen/riscv-vp.
It is not RISC-V specific and can be used to connect any GPIO peripheral, as long as it is modelled in C++.

It is partly described in [this paper](https://www.mdpi.com/2079-9268/12/4/52).
Tests can be built with the usual cmake pattern `mkdir build && cd build && cmake .. && make`.
