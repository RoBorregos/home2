import ChromaAdapter
from sentence_transformers import SentenceTransformer
from llama_cpp import Llama

llm = Llama.from_pretrained(
    repo_id="lmstudio-community/Qwen2.5-7B-Instruct-1M-GGUF",
    filename="Qwen2.5-7B-Instruct-1M-Q3_K_L.gguf",
    verbose=False,
)

client = ChromaAdapter()
model = SentenceTransformer("all-MiniLM-L6-v2")


def answer_question(question, top_k=3, threshold=0.4):
    # Define the existing collections you want to use
    collection_names = ["frida_knowledge", "roborregos_knowledge", "tec_knowledge"]

    all_context_pairs = []

    # Fetch from ChromaAdapter using each collection
    for collection_name in collection_names:
        try:
            collection = client.get_collection(collection_name)
            results = collection.query(
                query_texts=[question],
                n_results=top_k,
                include=["documents", "distances"],
            )

            documents = results.get("documents", [[]])[0]
            distances = results.get("distances", [[]])[0]

            # Pair scores with documents
            context_pairs = list(zip(distances, documents))
            all_context_pairs.extend(context_pairs)

        except ValueError as e:
            print(f"Warning: Collection {collection_name} not found. {str(e)}")
            continue

    # Sort all results by similarity score descending
    all_context_pairs.sort(key=lambda x: -x[0])

    # Select only top_k results
    top_contexts = all_context_pairs[:top_k]

    # Keep only relevant contexts (score > threshold)
    relevant_contexts = [doc for score, doc in top_contexts if score > threshold]

    if relevant_contexts:
        context_text = "\n".join(relevant_contexts)
        prompt = f"""You have the following information retrieved from knowledge bases:\n{context_text}\n\nUse it to answer the following question:\n{question}"""
    else:
        prompt = question  # Fallback to pure LLM if no good context

    # Query the local LLM
    response = llm.create_chat_completion(
        messages=[{"role": "user", "content": prompt}]
    )
    assistant_response = response["choices"][0]["message"]["content"]

    return {
        "response": assistant_response,
        "score": top_contexts[0][0] if top_contexts else 0,
    }


# Example usage
question = "What is the capital of France?"
answer = answer_question(question)
print(answer)

# 1. knowledge base upload of txt with embeddings per line to chroma db (there are 3 different knowledge bases)
# 2. when the query is done we will retrieve the top 3 results from all the files
# 3. we will return the answer and score to measure
# 4. if the score is not greater than 0.4 then use normal llm to answer the question (to target potential quizz questions and dates)
# 5. otherwise return the answer from the knowledge base and pass it on to the local llm for a structured output (knowledge base answer + local llm answer)
