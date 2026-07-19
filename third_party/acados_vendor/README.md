# acados_vendor

This package builds the repository-pinned `vendor/acados` source into the
current colcon install prefix. It excludes examples, tests, OpenMP, and optional
QP solvers; Flying Hand uses HPIPM through acados.

On x86-64, BLASFEO selects the local CPU implementation at build time so the
100-node controller can meet its 8 ms budget. Build deployment artifacts on the
target CPU (or an ISA-compatible build host); do not copy these shared libraries
to an older x86-64 processor.

```bash
git submodule update --init --recursive third_party/acados_vendor/vendor/acados
colcon build --packages-select acados_vendor
```
