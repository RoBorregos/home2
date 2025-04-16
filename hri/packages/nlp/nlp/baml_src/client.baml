
client<llm> R1 {
  provider "openai-generic"
  options {
    base_url "http://localhost:11434/v1"
    model "deepseek-r1:7b"
  }
}