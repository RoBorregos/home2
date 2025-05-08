#!/usr/bin/env python3

import numpy as np
import rclpy
from chroma_adapter import ChromaAdapter
from nlp.assets.dialogs import get_answer_question_dialog
from openai import OpenAI
from rclpy.node import Node

from frida_constants.hri_constants import MODEL, RAG_SERVICE
from frida_interfaces.srv import AnswerQuestion


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


class RAGService(Node):
    def __init__(self):
        super().__init__("rag_service")

        self.get_logger().info("Initializing RAG node")

        # Declare ROS parameters
        self.declare_parameter("api_key", "ollama")
        self.declare_parameter("base_url", "http://localhost:11434/v1")
        self.declare_parameter("model_name", MODEL.LLM_WRAPPER.value)
        self.declare_parameter("temperature", 0.0)
        self.declare_parameter("service_name", RAG_SERVICE)
        self.declare_parameter(
            "default_collections",
            ["frida_knowledge", "roborregos_knowledge", "tec_knowledge"],
        )
        self.declare_parameter("default_topk", 5)
        self.declare_parameter("default_threshold", 0.7)
        self.declare_parameter("embedding_keys", [])
        self.declare_parameter("similarity_threshold", 0.5)

        # Get parameter values
        self.api_key = self.get_parameter("api_key").value
        self.base_url = self.get_parameter("base_url").value
        self.model_name = self.get_parameter("model_name").value
        self.temperature = self.get_parameter("temperature").value
        self.service_name = self.get_parameter("service_name").value
        self.default_collections = self.get_parameter("default_collections").value
        self.default_topk = self.get_parameter("default_topk").value
        self.default_threshold = self.get_parameter("default_threshold").value
        self.embedding_keys = self.get_parameter("embedding_keys").value
        self.similarity_threshold = self.get_parameter("similarity_threshold").value

        # Setup Chroma Adapter
        self.client = ChromaAdapter()

        # Initialize LLM client
        self.llm = OpenAI(api_key=self.api_key, base_url=self.base_url)

        # Create the service
        self.srv = self.create_service(
            AnswerQuestion, self.service_name, self.answer_question_callback
        )
        self.get_logger().info(f"RAG node initialized with model: {self.model_name}")

    def compare_objects_recursively(self, obj1, obj2, embedding_model):
        """
        Recursively compares two objects and calculates cosine similarity between their values.
        Returns a score between 0 and 1, where 1 means perfect similarity.

        Args:
            obj1: First object to compare
            obj2: Second object to compare
            embedding_model: Model to use for embedding text

        Returns:
            float: Similarity score between 0 and 1
        """
        # Base case: both are primitive values (strings, numbers, etc.)
        if isinstance(obj1, (str, int, float, bool)) and isinstance(
            obj2, (str, int, float, bool)
        ):
            # For strings, use embedding similarity
            if isinstance(obj1, str) and isinstance(obj2, str):
                if not obj1.strip() and not obj2.strip():  # Both empty strings
                    return 1.0
                try:
                    emb1 = embedding_model.embed_text(str(obj1))
                    emb2 = embedding_model.embed_text(str(obj2))
                    return compute_cosine_similarity(emb1, emb2)
                except Exception as e:
                    self.get_logger().warn(f"Error computing embeddings: {e}")
                    return 1.0 if obj1 == obj2 else 0.0
            # For exact matches of other primitives
            return 1.0 if obj1 == obj2 else 0.0

        # Case: both are lists
        elif isinstance(obj1, list) and isinstance(obj2, list):
            if len(obj1) != len(obj2):
                return 0.0  # Different lengths

            if not obj1:  # Both empty
                return 1.0

            # Compare each element and average the similarities
            similarities = []
            for i in range(len(obj1)):
                if i < len(obj2):
                    similarities.append(
                        self.compare_objects_recursively(
                            obj1[i], obj2[i], embedding_model
                        )
                    )

            return sum(similarities) / len(similarities) if similarities else 0.0

        # Case: both are dictionaries
        elif isinstance(obj1, dict) and isinstance(obj2, dict):
            # Check if keys match
            keys1 = set(obj1.keys())
            keys2 = set(obj2.keys())
            if keys1 != keys2:
                # Calculate key overlap ratio
                common_keys = keys1.intersection(keys2)
                all_keys = keys1.union(keys2)
                if not all_keys:
                    return 1.0  # Both empty dicts
                overlap_ratio = len(common_keys) / len(all_keys)
                if (
                    overlap_ratio < self.similarity_threshold
                ):  # Less than threshold keys in common
                    return 0.0
            else:
                common_keys = keys1  # All keys match

            if not obj1:  # Both empty
                return 1.0

            # Compare each value and average the similarities
            similarities = []
            for key in common_keys:
                if key in self.embedding_keys:
                    # Always use embedding similarity for specified keys
                    try:
                        emb1 = embedding_model.embed_text(str(obj1[key]))
                        emb2 = embedding_model.embed_text(str(obj2[key]))
                        similarities.append(compute_cosine_similarity(emb1, emb2))
                    except Exception as e:
                        self.get_logger().warn(
                            f"Error computing embeddings for key {key}: {e}"
                        )
                        similarities.append(1.0 if obj1[key] == obj2[key] else 0.0)
                else:
                    # Recursive comparison for other keys
                    similarities.append(
                        self.compare_objects_recursively(
                            obj1[key], obj2[key], embedding_model
                        )
                    )

            return sum(similarities) / len(similarities) if similarities else 1.0

        # Different types
        else:
            return 0.0

    def answer_question_callback(self, request, response):
        question = request.question
        top_k = request.topk if request.topk else self.default_topk
        threshold = request.threshold if request.threshold else self.default_threshold
        collections = (
            request.collections if request.collections else self.default_collections
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
            best_score = top_contexts[0][0] if top_contexts else 0.0

            relevant_contexts = (
                [doc for score, doc in top_contexts if score > threshold]
                if best_score > threshold
                else []
            )

            self.get_logger().info(
                f"Generating LLM answer (threshold={threshold}, best_score={best_score})"
            )

            messages = get_answer_question_dialog(relevant_contexts, question)
            completion = self.llm.chat.completions.create(
                model=self.model_name,
                temperature=self.temperature,
                messages=messages,
            )
            assistant_response = completion.choices[0].message.content

            response.success = True
            response.response = assistant_response
            response.score = best_score

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
