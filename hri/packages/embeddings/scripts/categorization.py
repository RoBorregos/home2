#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pydantic import BaseModel, ValidationError
from typing import Optional
from enum import Enum
import json
from frida_interfaces.srv import (
    AddEntry,
    BuildEmbeddings,
    QueryEntry,
)

# Assuming ChromaAdapter handles Chroma client and embedding functions
from ChromaClient import ChromaClient


class MetadataProfile(str, Enum):
    ITEMS = "items"
    LOCATIONS = "locations"
    ACTIONS = "actions"


# Metadata validation model for metadata
class MetadataModel(BaseModel):
    shelve: Optional[str] = None
    category: Optional[str] = None
    default_location: Optional[str] = None
    context: Optional[str] = None

    # other metadata fields can be defined here
    @classmethod
    def with_profile(
        cls, profile: MetadataProfile = MetadataProfile.ITEMS, **overrides
    ):
        profiles = {
            MetadataProfile.ITEMS: {
                "context": "household items",
            },
            MetadataProfile.LOCATIONS: {
                "context": "house locations",
            },
            MetadataProfile.ACTIONS: {
                "context": "human actions",
            },
        }
        base = profiles.get(profile, {})
        data = {**base, **overrides}
        return cls(**data)


class Embeddings(Node):
    def __init__(self):
        super().__init__("embeddings")
        self.get_logger().info("Initializing categorization node.")

        # Parameters for services
        self.declare_parameter(
            "ADD_ENTRY_SERVICE", "/hri/nlp/embeddings/add_entry_service"
        )
        self.declare_parameter(
            "QUERY_ENTRY_SERVICE", "/hri/nlp/embeddings/query_entry_service"
        )
        self.declare_parameter(
            "BUILD_EMBEDDINGS_SERVICE", "/hri/nlp/embeddings/build_embeddings_service"
        )
        # Resolve parameters

        add_entry_service = (
            self.get_parameter("ADD_ENTRY_SERVICE").get_parameter_value().string_value
        )
        query_entry_service = (
            self.get_parameter("QUERY_ENTRY_SERVICE").get_parameter_value().string_value
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
            AddEntry, add_entry_service, self.add_entry_callback
        )
        self.query_entry_service = self.create_service(
            QueryEntry, query_entry_service, self.query_entry_callback
        )

        self.get_logger().info("Categorization node initialized.")

    def add_entry_callback(self, request, response):
        """Service callback to add items to ChromaDB"""
        try:
            # Ensure documents is a list
            if request.metadata:
                metadatas_ = json.loads(request.metadata)
            else:
                metadatas_ = request.metadata
            documents = (
                request.document
                if isinstance(request.document, list)
                else [request.document]
            )
            metadatas = metadatas_ if metadatas_ else [{} for _ in documents]

            metadata_objects = []

            # Normalize and validate all metadata entries using the profile
            for meta in metadatas:
                metadata_parsed = MetadataModel.with_profile(request.collection, **meta)
                metadata_objects.append(metadata_parsed.model_dump())

            # Inject context into documents and preserve original names
            for i, (doc, meta) in enumerate(zip(documents, metadata_objects)):
                meta["original_name"] = doc
                context = meta.get("context")
                if context:
                    documents[i] = f"{doc} {context}"

            # Send to Chroma
            self.chroma_adapter.add_entries(
                request.collection, documents, metadata_objects
            )

            response.success = True
            response.message = "Item(s) added successfully"

        except ValidationError as e:
            response.success = False
            response.message = f"Invalid metadata: {str(e)}"

        except Exception as e:
            response.success = False
            response.message = f"Failed to add item: {str(e)}"
            self.get_logger().error(response.message)

        return response

    def query_entry_callback(self, request, response):
        """Service callback to query items from ChromaDB"""
        try:
            # Delegate to ChromaAdapter to handle the actual ChromaDB interaction
            results = self.chroma_adapter.query(
                request.collection, request.query, request.topk
            )
            response.results = [
                str(doc) for doc in results
            ]  # Just loop through the list directly
            response.success = True if response.results else False
            response.message = (
                "Query successful" if response.results else "No matching items found"
            )
        except Exception as e:
            response.success = False
            response.message = f"Failed to query items: {str(e)}"
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
