#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pydantic import BaseModel, ValidationError
from typing import Optional
from frida_interfaces.srv import (
    AddItem,
    BuildEmbeddings,
    QueryItem,
    AddLocation,
    QueryLocation,
)

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

        # Parameters for services
        self.declare_parameter(
            "ADD_ITEM_SERVICE", "/hri/nlp/embeddings/add_item_service"
        )
        self.declare_parameter(
            "QUERY_ITEM_SERVICE", "/hri/nlp/embeddings/query_item_service"
        )
        self.declare_parameter(
            "BUILD_EMBEDDINGS_SERVICE", "/hri/nlp/embeddings/build_embeddings_service"
        )
        self.declare_parameter(
            "ADD_LOCATION_SERVICE", "/hri/nlp/embeddings/add_location_service"
        )
        self.declare_parameter(
            "QUERY_LOCATION_SERVICE", "/hri/nlp/embeddings/query_location_service"
        )
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
        add_location_service = (
            self.get_parameter("ADD_LOCATION_SERVICE")
            .get_parameter_value()
            .string_value
        )
        query_location_service = (
            self.get_parameter("QUERY_LOCATION_SERVICE")
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
        self.add_location_service = self.create_service(
            AddLocation, add_location_service, self.add_location_callback
        )
        self.query_location_service = self.create_service(
            QueryLocation, query_location_service, self.query_location_callback
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
                self.chroma_adapter.add_entries_with_metadata(
                    request.collection, request.document, metadata_parsed
                )
            else:
                # If metadata is empty (either empty string or only whitespace), set it to an empty dictionary
                metadata_parsed = {}
                self.chroma_adapter.add_entries(request.collection, request.document)
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
                "items", request.query, request.topk, request.return_location
            )
            if request.return_location:
                response.results = [str(doc) for doc in results.get("documents", [])]
                response.success = True if response.results else False
                response.message = (
                    "Query successful"
                    if response.results
                    else "No matching items found"
                )
                response.locations = [str(doc) for doc in results.get("locations", [])]
            else:
                response.results = [str(doc) for doc in results.get("documents", [])]
                response.success = True if response.results else False
                response.message = (
                    "Query successful"
                    if response.results
                    else "No matching items found"
                )

        except Exception as e:
            response.success = False
            response.message = f"Failed to query items: {str(e)}"
            self.get_logger().error(response.message)
        return response

    def query_location_callback(self, request, response):
        """Service callback to query locations from ChromaDB.
        arguments:
        request: (location str, topk int,return_coord: bool)
        response: (results: List[str], success: bool, message: str, coords: List[str])
        """
        try:
            # Delegate to ChromaAdapter to handle the actual ChromaDB interaction
            results = self.chroma_adapter.query(
                request.collection, request.query, request.topk
            )
            if request.return_coord:
                response.coords = [str(doc) for doc in results.get("coords", [])]
                response.results = [str(doc) for doc in results.get("documents", [])]
                response.success = True if response.results else False
                response.message = (
                    "Query successful"
                    if response.results
                    else "No matching items found"
                )

            else:
                response.results = [str(doc) for doc in results.get("documents", [])]
                response.success = True if response.results else False
                response.message = (
                    "Query successful"
                    if response.results
                    else "No matching items found"
                )

        except Exception as e:
            response.success = False
            response.message = f"Failed to query items: {str(e)}"
            self.get_logger().error(response.message)
        return response

    def add_location_callback(self, request, response):
        """Service callback to add locations to ChromaDB"""
        try:
            # Check if metadata is provided and if it's not empty
            if request.metadata.strip():
                # If metadata is provided and not empty, validate and parse it
                metadata_parsed = MetadataModel.model_validate_json(request.metadata)
                metadata_parsed = metadata_parsed.model_dump()
                self.chroma_adapter.add_entries_with_metadata(
                    request.collection, request.document, metadata_parsed
                )
            else:
                # If metadata is empty (either empty string or only whitespace), set it to an empty dictionary
                metadata_parsed = {}
                self.chroma_adapter.add_entries(request.collection, request.document)
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

    def build_embeddings_callback(self, request, response):
        """Method to build embeddings for the household items data"""

        try:
            # Call the build_embeddings_callback of ChromaAdapter to handle the actual embedding process
            if request.rebuild:
                self.get_logger().info("Rebuilding embeddings")
                self.chroma_adapter.remove_all_collections()
                self.chroma_adapter.build_embeddings()
            else:
                self.chroma_adapter.build_embeddings()
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
