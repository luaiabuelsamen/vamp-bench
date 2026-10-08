# Benchmark v2 freeze

- **Freeze commit:** `6aea6eecf76a173d7ebdf548104aaec854b6b6c7` ("Benchmark v2 FREEZE"). Its contents are listed in `docs/V2_PREREGISTRATION.md`.
- **Seeds:** `seed_from(H, label)` = first 8 hex digits of sha256("H:label"). VAMP uses `seed % 100000` as its Halton skip.

| label | seed |
|---|---|
| `gen_v2` | 660586511 |
| `select_mbm` | 755279182 |
| `select_mpinets` | 3943361810 |
| `select_v2` | 3645628491 |
| `seed0` | 1058006229 |
| `seed1` | 2892848303 |
| `seed2` | 3625993594 |
| `seed3` | 2115821575 |
| `seed4` | 275456021 |
| `seed5` | 1751982631 |
| `seed6` | 1265799964 |
| `seed7` | 3318612834 |
| `seed8` | 899135083 |
| `seed9` | 2490896690 |
