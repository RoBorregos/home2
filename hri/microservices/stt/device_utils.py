import ctranslate2


def detect_device_and_compute_type(preferred_device=None):
    """Detect the best device/compute type without requiring torch."""
    if preferred_device == "cpu":
        return "cpu", "int8"

    try:
        cuda_device_count = ctranslate2.get_cuda_device_count()
        if cuda_device_count > 0:
            supported_types = set(ctranslate2.get_supported_compute_types("cuda"))
            if "float16" in supported_types:
                return "cuda", "float16"
            if "int8_float16" in supported_types:
                return "cuda", "int8_float16"
            if "float32" in supported_types:
                return "cuda", "float32"
            return "cuda", "default"
    except Exception as exc:
        print(f"CUDA detection via ctranslate2 failed: {exc}")

    # Fallback for environments where torch is present but ctranslate2 probing fails.
    # try:
    #     import torch

    #     if torch.cuda.is_available():
    #         major, _ = torch.cuda.get_device_capability(0)
    #         return "cuda", "float16" if major >= 7 else "float32"
    # except Exception:
    #     pass

    # No CUDA device detected or probing unavailable.
    return "cpu", "int8"
