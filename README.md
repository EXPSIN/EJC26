# Quadratic-Programming-Based Cooperative Object Transport

MATLAB and interactive web simulations accompanying:

> S. Wu, Z. Qin, T. Liu, and Z.-P. Jiang, “Quadratic-programming-based control of multi-robot systems for cooperative object transport,” *European Journal of Control*, 2026, Article 101598.

[Paper](https://www.sciencedirect.com/science/article/pii/S0947358026001512) · [DOI](https://doi.org/10.1016/j.ejcon.2026.101598)

The controller uses quadratic programming to allocate contact forces among multiple robots and drive a transported object to follow a velocity command.

## Files

- [`main.m`](main.m): MATLAB simulation used in the paper.
- [`index.html`](index.html): mobile-oriented interactive web simulation.

## MATLAB

Requires MATLAB and Optimization Toolbox (`quadprog`). Run:

```matlab
main
```

In `closedloop_dynamics`, enable one position-control gain:

```matlab
k_p = 1.0;   % Parameter Set 1: nominal circular trajectory
% k_p = 0.1; % Parameter Set 2: degraded tracking
```

## Web Demo

Open [`index.html`](index.html) directly.

- **Mode 1:** tap or drag to set the direction and magnitude of $v_c$.
- **Mode 2:** reproduce the paper simulation and switch between the two parameter sets.

## Citation

```bibtex
@article{Wu-Qin-Liu-Jiang-EJC-2026,
title="Quadratic-programming-based control of multi-robot systems for cooperative object transport",
author="S. Wu and Z. Qin and T. Liu and Z.-P. Jiang",
journal="European Journal of Control",
pages="101598",
year=2026,
doi="10.1016/j.ejcon.2026.101598",
publisher="Elsevier"}
```

## Acknowledgments

Thanks to LLM for helping generate the interactive HTML simulation in [`index.html`](index.html).
