#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from frida_interfaces.srv import AnswerQuestion
from chroma_adapter import ChromaAdapter
from openai import OpenAI
from nlp.assets.dialogs import get_answer_question_dialog


class RAGService(Node):
    def __init__(self):
        super().__init__("rag_service")

        self.get_logger().info("Initializing RAG node")

        # Setup Chroma Adapter
        self.client = ChromaAdapter()

        self.api_key = "ollama"
        self.base_url = "http://localhost:11434/v1"
        self.model_name = "qwen2.5"
        self.temperature = 0.0

        self.llm = OpenAI(api_key=self.api_key, base_url=self.base_url)
        # Create the service
        self.srv = self.create_service(
            AnswerQuestion, "/hri/rag/answer_question", self.answer_question_callback
        )
        self.get_logger().info("RAG node initialized")

    def answer_question_callback(self, request, response):
        question = request.question
        top_k = request.topk
        threshold = request.threshold
        collections = (
            request.collections
            if request.collections
            else ["frida_knowledge", "roborregos_knowledge", "tec_knowledge"]
        )

        try:
            all_context_pairs = []

            for collection_name in collections:
                try:
                    collection = self.client.get_collection(collection_name)
                    results = collection.query(
                        query_texts=[question],
                        n_results=top_k,
                        include=["documents", "distances"],
                    )
                    documents = results.get("documents", [[]])[0]
                    distances = results.get("distances", [[]])[0]

                    context_pairs = list(zip(distances, documents))
                    all_context_pairs.extend(context_pairs)

                except Exception as e:
                    self.get_logger().warn(f"Collection {collection_name} error: {e}")

            all_context_pairs.sort(key=lambda x: -x[0])
            top_contexts = all_context_pairs[:top_k]
            relevant_contexts = [
                doc for score, doc in top_contexts if score > threshold
            ]

            self.get_logger().info("Generating LLM answer")

            messages = get_answer_question_dialog(relevant_contexts, question)
            completion = self.llm.chat.completions.create(
                model=self.model_name,
                temperature=self.temperature,
                messages=messages,
            )
            assistant_response = completion.choices[0].message.content

            response.success = True
            response.response = assistant_response
            response.score = top_contexts[0][0] if top_contexts else 0.0

            # Fill the service response
            response.success = True
            response.response = assistant_response
            response.score = top_contexts[0][0] if top_contexts else 0

        except Exception as e:
            self.get_logger().error(f"Error during RAG process: {e}")
            response.success = False
            response.response = f"Failed to answer question: {e}"
            response.score = 0.0

        return response


def main(args=None):
    rclpy.init(args=args)
    node = RAGService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
