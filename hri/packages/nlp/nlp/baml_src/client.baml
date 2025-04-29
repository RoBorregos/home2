
client<llm> R1 {
  provider "openai-generic"
  options {
    base_url "http://10.22.173.172:11434/v1"
    model "deepseek-r1:32b"
    max_tokens 4096
  }
}