# A single entry from the semantic map: a physically distinct object the
# robot has observed, deduplicated by label + position (see
# embeddings/scripts/postgres_adapter.py::upsert_semantic_object).
string label
float32 x
float32 y
float32 z
string frame_id
float32 confidence
string area
int32 observations
