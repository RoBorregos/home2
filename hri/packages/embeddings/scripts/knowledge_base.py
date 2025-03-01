import os
from sentence_transformers import SentenceTransformer, util

model = SentenceTransformer("all-MiniLM-L6-v2")

directory = "hri/packages/embeddings/embeddings/dataframes/knowledge_base"

file_embeddings = {}
for filename in os.listdir(directory):
    if filename.endswith(".txt"):
        with open(os.path.join(directory, filename), "r") as file:
            text = file.read()
            lines = text.split("\n")
            embeddings = model.encode(lines, convert_to_tensor=True)
            file_embeddings[filename] = (lines, embeddings)


def answer_question(question):
    question_embedding = model.encode(question, convert_to_tensor=True)
    all_similarities = []

    for filename, (lines, embeddings) in file_embeddings.items():
        similarities = util.pytorch_cos_sim(question_embedding, embeddings)
        for i, similarity in enumerate(similarities[0]):
            all_similarities.append((similarity.item(), lines[i], filename))

    # Sort by similarity and get the top 3 results
    all_similarities.sort(key=lambda x: x[0], reverse=True)
    top_3_results = all_similarities[:3]

    return top_3_results


# Example usage
question = "what is frida"
top_3_results = answer_question(question)
for similarity, line, filename in top_3_results:
    print(f"File: {filename}, Similarity: {similarity}, Line: {line}")


# 1. kwoledge base upload of txt with embeddings per line to chroma db (there are 3 different knowledge bases)
# 2. when the query is done we will retrieve the top 3 results from all the files
# 3. we will return the answer and score to measure
# 4. if the score is not greater than 0.4 then use normal llm to answer the question (to target potential quizz questions and dates)
# 5. otherwise return the answer from the knowledge base and pass it on to the local llm for a structured output (knowledge base answer + local llm answer)
