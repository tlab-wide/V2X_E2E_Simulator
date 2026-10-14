# Noise Profiles

| | |
|:--|:--|
| File | `config/defaults/noise.yaml` |
| Web viewer | Settings → **Defaults** |
| Unity export | *V2X → Config → Export Scene To Config* |

These defaults are shared by all scenarios.

```yaml
noise_profiles:
- name: Default
  distribution: uniform        # uniform | gaussian
  x_m: 0.5
  y_m: 0.5
  z_m: 0.5
  gaussian_mean: 0
  gaussian_std_dev: 25
```

Publishers refer to a profile by `name` for each kind of noise: `position`, `rotation`, `dimension` (size) and `probability` (existence).

| Field | Meaning |
|:--|:--|
| `distribution` | `uniform`: a random value in [−amplitude, +amplitude] per axis. `gaussian`: `gaussian_mean + gaussian_std_dev × amplitude × N(0,1)` |
| `x_m`, `y_m`, `z_m` | amplitude per axis (m) |

`NoNoise` (all zero) is used for ground truth.
