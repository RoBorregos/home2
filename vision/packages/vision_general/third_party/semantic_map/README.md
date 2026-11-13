# Semantic Map library

The logic behind `scripts/task_nodes/semantic_map_node.py`: a persistent graph of
rooms, the furniture in them and the objects seen on it. `core/` imports `numpy` and
`ports.py` only, never ROS, so it is tested without a robot; `adapters/ros.py` is
the one file that talks to ROS.

**This is a copy, edited here.** It comes from
[RoBorregos/Semantic-Map](https://github.com/RoBorregos/Semantic-Map) at `f8b2961`,
which also holds what the robot does not need: the Midterm report, `eval/`, the
photos, and the local Ollama and CLIP adapters.

Changed since that commit, for building the map online:

- A failed crop embedding no longer duplicates objects: text and crop vectors are
  never compared with each other.
- Position, embedding, label and confidence are fused instead of overwritten.
- Objects answer queries after `confirm_min_observations` detections, and are
  forgotten after `miss_limit` looks without detecting them (`core/visibility.py`
  decides what counts as a look).
- Surfaces are measured at the furniture, `surface_standoff_m` ahead of the tagged pose.
- Without calibration, a command for something absent is `not_found`, not the best match.
- Malformed map entries are skipped, and an LLM or service failure degrades or times
  out instead of hanging.

```bash
cd vision/packages/vision_general/third_party/semantic_map
PYTHONPATH=. python3 -m pytest tests
```
