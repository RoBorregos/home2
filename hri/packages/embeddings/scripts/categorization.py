#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pydantic import BaseModel, ValidationError
from typing import Optional
from enum import Enum
import json
import pandas as pd
from pathlib import Path
from frida_interfaces.srv import (
    AddEntry,
    BuildEmbeddings,
    QueryEntry,
)

# Assuming ChromaAdapter handles Chroma client and embedding functions
from ChromaAdapter import ChromaAdapter


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
        self.chroma_adapter = ChromaAdapter()

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
            results_raw = self.chroma_adapter.query(
                request.collection, request.query, request.topk
            )

            docs = results_raw.get("documents", [[]])[0]
            metas = results_raw.get("metadatas", [[]])[0]

            response.results = [
                meta.get("original_name", doc) if isinstance(meta, dict) else doc
                for doc, meta in zip(docs, metas)
            ]

            response.success = bool(response.results)
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
                self.build_embeddings()
            else:
                self.build_embeddings()
            response.success = True
            response.message = "Embeddings built successfully"
            self.get_logger().info("Build request handled successfully")

        except Exception as e:
            response.success = False
            response.message = f"Error while building embeddings: {str(e)}"
            self.get_logger().error(f"Error while building embeddings: {str(e)}")

        return response

    def build_embeddings(self):
        """
        Method to build embeddings for household use.
        Reads CSV files from the designated dataframes folder,
        and for each file:
        - Reads documents and (if available) metadata.
        - Gets or creates a corresponding collection.
        - Adds entries to the collection via the add_entries method,
            which will process documents and metadata (adding "original_name",
            appending "context", and cleaning metadata) automatically.
        """
        # Get the directory of the current script
        script_dir = Path(__file__).resolve().parent
        # Define the folder where the CSV files are located
        dataframes_folder = script_dir / "../embeddings/dataframes"

        # Ensure the folder exists
        if not (dataframes_folder.exists() and dataframes_folder.is_dir()):
            raise FileNotFoundError(
                f"The folder {dataframes_folder} does not exist or is not a directory."
            )

        # Get all CSV files in the folder
        dataframes = [
            file.resolve()
            for file in dataframes_folder.iterdir()
            if file.suffix == ".csv"
        ]
        # Check if there are any CSV files
        if not dataframes:
            raise FileNotFoundError(
                f"No CSV files found in the folder {dataframes_folder}."
            )
        collections = {}
        for file in dataframes:
            print("Processing file:", file)
            # Read the CSV file into a pandas DataFrame
            df = pd.read_csv(file)

            # Ensure that the 'documents' column exists
            if "documents" not in df.columns:
                raise ValueError(f"The 'documents' column is missing in {file}")

            documents = df["documents"].tolist()

            # Process metadata if available; otherwise, use None
            metadatas_ = None
            if "metadata" in df.columns:
                metadatas_ = self.chroma_adapter.json2dict(df["metadata"])
                # Process each document/metadata pair
                [documents, metadatas_] = self.add_basics(documents, metadatas_)

            # Sanitize and get or create the collection
            collection_name = self.chroma_adapter._sanitize_collection_name(file.stem)
            collections[collection_name] = (
                self.chroma_adapter._get_or_create_collection(collection_name)
            )

            self.chroma_adapter.add_entries(collection_name, documents, metadatas_)

        return

    def add_basics(self, documents, metadatas):
        for i, (doc, meta) in enumerate(zip(documents, metadatas)):
            # Add "original_name" to each metadata dictionary
            meta["original_name"] = doc
            # If a "context" is provided, append it to the document
            if "context" in meta and meta["context"]:
                documents[i] = f"{doc} {meta['context']}"
        return documents, metadatas


def main():
    rclpy.init()
    embeddings = Embeddings()
    rclpy.spin(embeddings)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
