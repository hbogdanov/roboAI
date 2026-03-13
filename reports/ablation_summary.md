# Ablation Summary

## Questions

- Does smarter frontier scoring improve coverage and utility?
- Does hybrid planning improve resilience under disturbances?
- Do semantics help when sensing and localization are noisy?
- Does two-robot exploration reduce time-to-coverage?

## Frontier Policy

| variant | success rate | mean coverage | mean runtime (s) | mean replans | mean recovery events | time-to-coverage | overlap | near-conflicts |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| information_gain | 1.000 | 0.652 | 12.93 | 6.00 | 3.00 | 161.0 | 0.00 | 0.00 |
| learned_linear | 1.000 | 0.653 | 7.51 | 4.00 | 1.00 | 137.0 | 0.00 | 0.00 |
| naive | 1.000 | 0.651 | 9.26 | 4.00 | 1.00 | 127.0 | 0.00 | 0.00 |
| semantic_information_gain | 1.000 | 0.652 | 8.84 | 6.00 | 3.00 | 161.0 | 0.00 | 0.00 |

## Disturbance Planner

| variant | success rate | mean coverage | mean runtime (s) | mean replans | mean recovery events | time-to-coverage | overlap | near-conflicts |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| astar | 1.000 | 0.652 | 7.14 | 4.00 | 1.00 | 139.0 | 0.00 | 0.00 |
| hybrid | 1.000 | 0.652 | 8.28 | 4.00 | 1.00 | 139.0 | 0.00 | 0.00 |

## Noise Semantics

| variant | success rate | mean coverage | mean runtime (s) | mean replans | mean recovery events | time-to-coverage | overlap | near-conflicts |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| disabled | 0.000 | 0.634 | 7.40 | 7.00 | 1.00 | 86.0 | 0.00 | 0.00 |
| enabled | 0.000 | 0.634 | 7.48 | 7.00 | 1.00 | 86.0 | 0.00 | 0.00 |

## Robot Count

| variant | success rate | mean coverage | mean runtime (s) | mean replans | mean recovery events | time-to-coverage | overlap | near-conflicts |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| multi | 1.000 | 0.753 | 2.94 | 2.00 | 0.00 | 26.5 | 0.99 | 0.00 |
| single | 1.000 | 0.652 | 8.92 | 6.00 | 3.00 | 161.0 | 0.00 | 0.00 |

