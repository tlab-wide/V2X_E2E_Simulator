# Network Profiles

| | |
|:--|:--|
| File | `config/defaults/network.yaml` |
| Web viewer | Settings → **Defaults** |
| Unity export | *V2X → Config → Export Scene To Config* |

These defaults are shared by all scenarios.

A network profile publishes a delayed, lossy copy of a topic as `<topic><topic_suffix>`, for example `/v2x/object_info/DSRC`. Publishers list the profiles they use under `network`.

```yaml
network_profiles:
- name: DSRC
  topic_suffix: /DSRC
  delay_ms: 100
  delay_variance_ms: 0
  packet_loss: 0.05
  packet_loss_variance: 0
  ground_truth: false
```

| Field | Meaning |
|:--|:--|
| `delay_ms`, `delay_variance_ms` | latency and its random ± part |
| `packet_loss`, `packet_loss_variance` | probability (0–1) that a message is dropped |
| `ground_truth` | also make a copy of the ground-truth topic |
