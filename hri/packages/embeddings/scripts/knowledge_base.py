#!/usr/bin/env python3

import rclpy
from embeddings.postgres_adapter import PostgresAdapter
from nlp.assets.dialogs import clean_question_rag, get_answer_question_dialog
from openai import OpenAI
from rclpy.node import Node

from frida_constants.hri_constants import KNOWLEDGE_TYPE, MODEL, RAG_SERVICE
from frida_interfaces.srv import AnswerQuestion


class RAGService(Node):
    def __init__(self):
        super().__init__("rag_service")

        self.get_logger().info("Initializing RAG node")

        # Declare ROS parameters
        self.declare_parameter("api_key", "ollama")
        self.declare_parameter("base_url", "http://localhost:11434/v1")
        self.declare_parameter("temperature", 0.0)
        self.declare_parameter("service_name", RAG_SERVICE)
        self.declare_parameter(
            "default_collections",
            [
                KNOWLEDGE_TYPE.TEC.value,
                KNOWLEDGE_TYPE.ROBORREGOS.value,
                KNOWLEDGE_TYPE.FRIDA.value,
            ],
        )
        self.declare_parameter("default_topk", 5)
        self.declare_parameter("default_threshold", 0.2)
        self.declare_parameter("embedding_keys", [])

        # Get parameter values
        self.api_key = self.get_parameter("api_key").value
        self.base_url = self.get_parameter("base_url").value
        self.temperature = self.get_parameter("temperature").value
        self.service_name = self.get_parameter("service_name").value
        self.default_collections = self.get_parameter("default_collections").value
        self.default_topk = self.get_parameter("default_topk").value
        self.default_threshold = self.get_parameter("default_threshold").value
        self.embedding_keys = self.get_parameter("embedding_keys").value

        self.model_name = MODEL.LLM_WRAPPER.value

        # Initialte postgres adapter
        self.pg = PostgresAdapter()
        # Initialize LLM client
        self.llm = OpenAI(api_key=self.api_key, base_url=self.base_url)

        # Create the service
        self.srv = self.create_service(
            AnswerQuestion, self.service_name, self.answer_question_callback
        )
        self.get_logger().info(f"RAG node initialized with model: {self.model_name}")

    def clean_question(self, question):
        """
        Cleans the question to ensure it is in a proper format for the LLM, before RAG
        """
        messages = clean_question_rag(question)

        response = self.llm.beta.chat.completions.parse(
            messages=messages, model="qwen2.5"
        )
        cleaned_question = response.choices[0].message.content.strip()
        if "</think>" in cleaned_question:
            cleaned_question = cleaned_question.split("</think>")[-1].strip()

        return cleaned_question

    def answer_question_callback(self, request, response):
        try:
            question = request.question  # self.clean_question(request.question)
            top_k = request.topk if request.topk else self.default_topk
            threshold = (
                request.threshold if request.threshold else self.default_threshold
            )
            collections = (
                request.knowledge_type
                if len(request.knowledge_type) > 0
                else self.default_collections
            )
            relevant_contexts = self.pg.get_context_from_knowledge(
                prompt=question,
                knowledge_type=collections,
                threshold=threshold,
                top_k=top_k,
            )

            if len(relevant_contexts) == 0:
                relevant_contexts = [
                    "We are RoBorregos, the official robotics team of Tec de Monterrey, made up of 53 talented students specializing in autonomous robotics, IoT, and web development.",
                    "We were founded in August 2015 and actively participate in national and international robotics competitions such as TMR, RoboCup, and IEEE LARC.",
                    "Our team's name is RoBorregos.",
                    "We proudly represent Mexico in robotics competitions.",
                    "We are affiliated with Tecnológico de Monterrey (Tec de Monterrey), a private research university in Mexico.",
                    "We hold events like TechTalks, Candidates, and RoboChamp to engage with students and the community.",
                    "We were inspired by a 2015 experience at TMR that showed us Mexico’s potential in robotics.",
                    "Our team values open source, good documentation, knowledge sharing, and active community engagement.",
                ]
            else:
                relevant_contexts = [k.text for k in relevant_contexts]

            messages = get_answer_question_dialog(relevant_contexts, question)

            completion = self.llm.beta.chat.completions.parse(
                model=self.model_name,
                temperature=self.temperature,
                messages=messages,
            )
            assistant_response = completion.choices[0].message.content

            if "</think>" in assistant_response:
                assistant_response = assistant_response.split("</think>")[-1].strip()

            assistant_response = assistant_response.replace("*", "")

            response.success = True
            response.response = assistant_response
            response.score = 1.0

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
