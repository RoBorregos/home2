
client<llm> R1 {
  provider "openai-generic"
  options {
    base_url "http://localhost:11434/v1"
    model "deepseek-r1:32b"
    max_tokens 4096
  }
}

client<llm> QWEN3_14B { 
  provider "openai-generic"
  options {
    base_url "http://localhost:11434/v1"
    model "qwen3:14b"
    max_tokens 4096
  }
}

client<llm> QWEN3_0_6B { 
  provider "openai-generic"
  options {
    base_url "http://localhost:11434/v1"
    model "qwen3:0.6b"
    max_tokens 4096
  }
}

client<llm> LOCAL_FINETUNED_NEW { 
  provider "openai-generic"
  options {
    base_url "http://localhost:11434/v1"
    model "rbrgs:latest"
    max_tokens 4096
  }
}

client<llm> PRO_2_5 {
  provider "openai-generic"
  retry_policy Constant
  options {
    base_url https://openrouter.ai/api/v1
    api_key env.OPENROUTER_API_KEY
    model "google/gemini-2.5-pro-preview-03-25"
    max_tokens 4096
  }
}

client<llm> FLASH_2_5 {
  provider "openai-generic"
  retry_policy Constant
  options {
    base_url https://openrouter.ai/api/v1
    api_key env.OPENROUTER_API_KEY
    model "google/gemini-2.5-flash-preview"
    max_tokens 4096
  }
}

client<llm> O4_MINI {
  provider "openai"
  retry_policy Constant
  options {
    api_key env.OPENAI_API_KEY
    model "o4-mini"
  }
}

client<llm> GPT_4_1_MINI {
  provider "openai"
  retry_policy Constant
  options {
    api_key env.OPENAI_API_KEY
    model "gpt-4.1-mini"
  }
}

client<llm> API_QWEN3_4B {
  provider "openai-generic"
  retry_policy Constant
  options {
    base_url https://openrouter.ai/api/v1
    api_key env.OPENROUTER_API_KEY
    model "qwen/qwen3-4b:free"
    max_tokens 4096
  }
}

client<llm> API_QWEN3_14B {
  provider "openai-generic"
  retry_policy Constant
  options {
    base_url https://openrouter.ai/api/v1
    api_key env.OPENROUTER_API_KEY
    model "qwen/qwen3-14b"
    max_tokens 4096
  }
}


retry_policy Constant {
  max_retries 4
  // Strategy is optional
  strategy {
    type constant_delay
    delay_ms 200
  }
}