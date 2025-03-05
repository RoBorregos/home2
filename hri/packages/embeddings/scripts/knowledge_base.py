import os
import chromadb
from sentence_transformers import SentenceTransformer, util
from llama_cpp import Llama

llm = Llama.from_pretrained(
    repo_id="lmstudio-community/Qwen2.5-7B-Instruct-1M-GGUF",
    filename="Qwen2.5-7B-Instruct-1M-Q3_K_L.gguf",
    verbose=False,
)

model = SentenceTransformer("all-MiniLM-L6-v2")
client = chromadb.Client()

frida_knowledge = client.create_collection("frida_knowledge")
roborregos_knowledge = client.create_collection("roborregos_knowledge")
tec_knowledge = client.create_collection("tec_knowledge")

directory = "hri/packages/embeddings/embeddings/dataframes/knowledge_base"

file_embeddings = {}
for filename in os.listdir(directory):
    if filename.endswith(".txt"):
        with open(os.path.join(directory, filename), "r") as file:
            text = file.read()
            lines = text.split("\n")
            embeddings = model.encode(lines, convert_to_tensor=True)
            file_embeddings[filename] = (lines, embeddings)

            # Upload embeddings to Chroma collections
            for i, line in enumerate(lines):
                embedding = embeddings[i].tolist()
                frida_knowledge.add_document(line, embedding)
                roborregos_knowledge.add_document(line, embedding)
                tec_knowledge.add_document(line, embedding)


def answer_question(question):
    question_embedding = model.encode(question, convert_to_tensor=True)
    all_similarities = []

    for filename, (lines, embeddings) in file_embeddings.items():
        similarities = util.pytorch_cos_sim(question_embedding, embeddings)
        for i, similarity in enumerate(similarities[0]):
            all_similarities.append((similarity.item(), lines[i], filename))

    # Get top 3 results
    all_similarities.sort(key=lambda x: x[0], reverse=True)
    top_3_results = all_similarities[:3]

    # Check if any similarity is above 0.4
    relevant_contexts = [result for result in top_3_results if result[0] > 0.4]

    # Answer with context from knowledge base
    if relevant_contexts:
        context_statements = " ".join([context[1] for context in relevant_contexts])
        prompt = f"{context_statements} {question}"
    # Answer with question only
    else:
        prompt = question

    response = llm.create_chat_completion(
        messages=[{"role": "user", "content": prompt}]
    )

    assistant_response = response["choices"][0]["message"]["content"]
    return assistant_response


# Example usage
question = "What is the capital of France?"
answer = answer_question(question)
print(answer)

# 1. knowledge base upload of txt with embeddings per line to chroma db (there are 3 different knowledge bases)
# 2. when the query is done we will retrieve the top 3 results from all the files
# 3. we will return the answer and score to measure
# 4. if the score is not greater than 0.4 then use normal llm to answer the question (to target potential quizz questions and dates)
# 5. otherwise return the answer from the knowledge base and pass it on to the local llm for a structured output (knowledge base answer + local llm answer)
