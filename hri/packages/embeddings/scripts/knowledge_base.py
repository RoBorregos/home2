#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from frida_interfaces.hri.srv import AnswerQuestion
from ChromaAdapter import ChromaAdapter
from sentence_transformers import SentenceTransformer
from llama_cpp import Llama


class RAGService(Node):
    def __init__(self):
        super().__init__("rag_service")

        self.get_logger().info("Initializing RAG Service...")

        # Setup Chroma Adapter
        self.client = ChromaAdapter()

        # Load Sentence Transformer
        self.model = SentenceTransformer("all-MiniLM-L6-v2")

        # Load Local LLM
        self.llm = Llama.from_pretrained(
            repo_id="lmstudio-community/Qwen2.5-7B-Instruct-1M-GGUF",
            filename="Qwen2.5-7B-Instruct-1M-Q3_K_L.gguf",
            verbose=False,
        )

        # Create the service
        self.srv = self.create_service(
            AnswerQuestion, "/hri/rag/answer_question", self.answer_question_callback
        )
        self.get_logger().info("RAG Service is ready!")

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

            if relevant_contexts:
                context_text = "\n".join(relevant_contexts)
                prompt = f"Use the following knowledge base information:\n{context_text}\n\nAnswer the question:\n{question}"
            else:
                prompt = question

            llm_response = self.llm.create_chat_completion(
                messages=[{"role": "user", "content": prompt}]
            )

            assistant_response = llm_response["choices"][0]["message"]["content"]

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
