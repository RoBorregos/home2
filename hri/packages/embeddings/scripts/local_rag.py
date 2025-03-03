from llama_cpp import Llama

llm = Llama.from_pretrained(
    repo_id="lmstudio-community/Qwen2.5-7B-Instruct-1M-GGUF",
    filename="Qwen2.5-7B-Instruct-1M-Q3_K_L.gguf",
)


llm.create_chat_completion(
    messages=[{"role": "user", "content": "What is the capital of France?"}]
)
