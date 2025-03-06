#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pydantic import BaseModel, ValidationError
from typing import Optional
from frida_interfaces.srv import AddItem, BuildEmbeddings, QueryItem

# Assuming ChromaAdapter handles Chroma client and embedding functions
from ChromaClient import ChromaClient


# Metadata validation model for metadata
class MetadataModel(BaseModel):
    shelve: Optional[str]
    category: Optional[str]
    timestamp: Optional[str]
    # other metadata fields can be defined here


class Embeddings(Node):
    def __init__(self):
        super().__init__("embeddings")
        self.get_logger().info("Initializing item_categorization.")

        # Declare parameters
        self.declare_parameter("Embeddings_model", "all-MiniLM-L12-v2")
        self.declare_parameter("collections_built", 0)

        # Parameters for services
        self.declare_parameter("ADD_ITEM_SERVICE", "add_item")
        self.declare_parameter("QUERY_ITEM_SERVICE", "query_item")
        self.declare_parameter("BUILD_EMBEDDINGS_SERVICE", "build_embeddings")

        # Resolve parameters

        add_item_service = (
            self.get_parameter("ADD_ITEM_SERVICE").get_parameter_value().string_value
        )
        query_item_service = (
            self.get_parameter("QUERY_ITEM_SERVICE").get_parameter_value().string_value
        )
        build_embeddings_service = (
            self.get_parameter("BUILD_EMBEDDINGS_SERVICE")
            .get_parameter_value()
            .string_value
        )
        # Initialize ChromaAdapter (handles Chroma client and embedding functions)
        self.chroma_adapter = ChromaClient()

        # Create services
        self.build_embeddings_service = self.create_service(
            BuildEmbeddings, build_embeddings_service, self.build_embeddings_callback
        )
        self.add_item_service = self.create_service(
            AddItem, add_item_service, self.add_item_callback
        )
        self.query_item_service = self.create_service(
            QueryItem, query_item_service, self.query_item_callback
        )
        self.get_logger().info("item_categorization initialized.")

    def add_item_callback(self, request, response):
        """Service callback to add items to ChromaDB"""
        try:
            # Check if metadata is provided and if it's not empty
            if request.metadata.strip():
                # If metadata is provided and not empty, validate and parse it
                metadata_parsed = MetadataModel.model_validate_json(request.metadata)
                metadata_parsed = metadata_parsed.model_dump()
                self.chroma_adapter.add_entry(
                    request.collection, request.document, metadata_parsed
                )
            else:
                # If metadata is empty (either empty string or only whitespace), set it to an empty dictionary
                metadata_parsed = {}
                self.chroma_adapter.add_entry(request.collection, request.document)
            # Delegate to ChromaAdapter to handle the actual ChromaDB interaction
            response.success = True
            response.message = "Item added successfully"
        except ValidationError as e:
            response.success = False
            response.message = f"Invalid metadata: {str(e)}"
        except Exception as e:
            response.success = False
            response.message = f"Failed to add item: {str(e)}"
            self.get_logger().error(response.message)
        return response

    def query_item_callback(self, request, response):
        """Service callback to query items from ChromaDB"""
        try:
            # Delegate to ChromaAdapter to handle the actual ChromaDB interaction
            results = self.chroma_adapter.query(
                request.collection, request.query, request.topk
            )

            response.results = [str(doc) for doc in results.get("documents", [])]
            response.success = True if response.results else False
            response.message = (
                "Query successful" if response.results else "No matching items found"
            )
            response.metadata = [str(doc) for doc in results.get("metadatas", [])]

        except Exception as e:
            response.success = False
            response.message = f"Failed to query items: {str(e)}"
            self.get_logger().error(response.message)
        return response

    def build_embeddings_callback(self, request, response):
        """Method to build embeddings for the household items data"""

        try:
            # Call the build_embeddings_callback of ChromaAdapter to handle the actual embedding process
            self.chroma_adapter.build_embeddings_callback()
            self.chroma_adapter.build_embeddings_callback()
            response.success = True
            response.message = "Embeddings built successfully"
            self.get_logger().info("Build request handled successfully")

        except Exception as e:
            response.success = False
            response.message = f"Error while building embeddings: {str(e)}"
            self.get_logger().error(f"Error while building embeddings: {str(e)}")

        return response


def main():
    rclpy.init()
    embeddings = Embeddings()
    rclpy.spin(embeddings)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
