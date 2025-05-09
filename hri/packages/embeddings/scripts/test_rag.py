#!/usr/bin/env python3
import json
import numpy as np
from sentence_transformers import SentenceTransformer, util
from openai import OpenAI


def compute_cosine_similarity(vec1, vec2):
    """
    Computes the cosine similarity between two vectors.

    Args:
        vec1: First vector
        vec2: Second vector

    Returns:
        float: Similarity score between 0 and 1
    """
    if np.all(vec1 == 0) or np.all(vec2 == 0):
        return 0.0  # Handle zero vectors

    dot_product = np.dot(vec1, vec2)
    norm_vec1 = np.linalg.norm(vec1)
    norm_vec2 = np.linalg.norm(vec2)
    similarity = dot_product / (norm_vec1 * norm_vec2)

    # Handle potential numerical issues
    return float(np.clip(similarity, 0.0, 1.0))


class LocalRAGService:
    def __init__(self, knowledge_base_file, api_key, base_url):
        """
        Initializes the Local RAG Service.

        Args:
            knowledge_base_file (str): Path to the JSON knowledge base file.
            api_key (str): API key for the LLM.
            base_url (str): Base URL for the LLM API.
        """
        self.knowledge_base_file = knowledge_base_file
        self.llm = OpenAI(api_key=api_key, base_url=base_url)
        self.model = SentenceTransformer("all-MiniLM-L6-v2")
        self.file_embeddings = self.load_knowledge_base()

    def load_knowledge_base(self):
        """
        Loads the knowledge base from a JSON file and computes embeddings.

        Returns:
            list: A list of dictionaries containing documents, metadata, and embeddings.
        """
        with open(self.knowledge_base_file, "r") as file:
            knowledge_base = json.load(file)

        for entry in knowledge_base:
            document = entry["document"]
            entry["embedding"] = self.model.encode(document, convert_to_tensor=True)

        return knowledge_base

    def clean_question(self, question):
        """
        Cleans the question using the LLM.

        Args:
            question (str): The original question.

        Returns:
            str: The cleaned question.
        """
        messages = [
            {
                "role": "system",
                "content": (
                    "You will be given a question. Your task is to clean it in a way of question format. "
                    "Return a question, ignore unnecessary or irrelevant details to the question."
                ),
            },
            {"role": "user", "content": question},
        ]

        response = self.llm.chat.completions.create(messages=messages, model="qwen3")
        cleaned_question = response.choices[0].message.content.strip()
        return cleaned_question

    def answer_question(self, question, top_k=5, threshold=0.1):
        """
        Answers a question using the local knowledge base.

        Args:
            question (str): The question to answer.
            top_k (int): Number of top results to consider.
            threshold (float): Minimum similarity score to consider relevant.

        Returns:
            str: The assistant's response.
        """
        # Step 1: Clean the question
        cleaned_question = self.clean_question(question)

        # Step 2: Encode the cleaned question
        question_embedding = self.model.encode(cleaned_question, convert_to_tensor=True)
        all_similarities = []

        # Compare the question embedding with each document in the knowledge base
        for entry in self.file_embeddings:
            similarity = util.pytorch_cos_sim(question_embedding, entry["embedding"])[
                0
            ][0].item()
            all_similarities.append((similarity, entry["document"], entry["metadata"]))

        # Get the top results
        all_similarities.sort(key=lambda x: x[0], reverse=True)
        top_results = all_similarities[:top_k]

        # Filter results with a similarity score above the threshold
        relevant_contexts = [result for result in top_results if result[0] > threshold]

        # Prepare the context for the LLM
        print("Cleaned question:", cleaned_question)

        if relevant_contexts:
            context_statements = " ".join([context[1] for context in relevant_contexts])
            user_prompt = f"{context_statements} {cleaned_question}"
        else:
            user_prompt = cleaned_question

        # Add FRIDA logic to the prompt
        from datetime import datetime, timedelta

        now = datetime.now()
        tomorrow = now + timedelta(days=1)
        current_time = now.strftime("%Y-%m-%d %H:%M:%S")
        tomorrow_time = tomorrow.strftime("%Y-%m-%d")
        day_of_week = now.strftime("%A")
        day_of_month = now.strftime("%d")

        system_prompt = (
            "You are FRIDA, a warm, efficient, and helpful robot assistant that lives in a smart home. "
            "Your purpose is to assist and host guests naturally, always responding politely and directly. "
            "Do not include any '<think>' or 'thinking' sections in your responses. "
            "When given a task, ignore the setting, names, gestures, or commands in the phrasing — focus only on the actual question or request. "
            "Use the provided context if available, and do not mention the source of your knowledge or that it came from any documents. Do not explain your reasoning. "
            "Answer clearly, naturally, and in a friendly tone. If the prompt suggests interaction (e.g., greeting someone, answering a quiz), respond accordingly as if you're speaking directly to that person. "
            "If asked something about yourself, you may share a short fun fact (e.g., “I'm FRIDA, your home assistant — always here to help!”). "
            "If no answer can be given based on the context, you may politely respond that you don't know at the moment. "
            f"Tomorrow: {tomorrow_time}\n\n"
            f"Day of the week: {day_of_week}\n\n"
            f"Day of the month: {day_of_month}\n\n"
            f"Current time: {current_time}\n\n"
            "Don't ask additional questions or ask for clarifications, just answer the question."
        )

        # Combine system and user prompts
        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_prompt},
        ]

        # Step 3: Generate the response using the LLM
        response = self.llm.chat.completions.create(messages=messages, model="qwen2.5")
        assistant_response = response.choices[0].message.content.strip()

        # Step 4: Remove think section
        if "<think>" in assistant_response:
            assistant_response = assistant_response.split("</think>")[-1].strip()
        return assistant_response


if __name__ == "__main__":
    # Path to the JSON knowledge base file
    knowledge_base_file = "roborregos_knowledge.json"

    # Initialize the Local RAG Service
    rag_service = LocalRAGService(
        knowledge_base_file=knowledge_base_file,
        api_key="ollama",
        base_url="http://100.108.245.54:11434/v1",
    )

    # Example question
    question = "how many members are in the team?"
    answer = rag_service.answer_question(question)
    print("Answer:", answer)
