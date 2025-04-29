#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pydantic import BaseModel, ValidationError
from typing import Optional, ClassVar, Dict
from enum import Enum
import json
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
    context: Optional[str] = ""
    complement: Optional[str] = None
    characteristic: Optional[str] = None
    result: Optional[str] = None
    status: Optional[int] = None
    timestamp: Optional[str] = None
    subarea: Optional[str] = None

    PROFILES: ClassVar[Dict[MetadataProfile, Dict[str, str]]] = {
        MetadataProfile.ITEMS: {"context": " item for household use"},
        MetadataProfile.LOCATIONS: {"context": " house locations"},
        MetadataProfile.ACTIONS: {"context": " human actions"},
    }

    @classmethod
    def with_profile(
        cls, profile: MetadataProfile = MetadataProfile.ITEMS, **overrides
    ):
        base = cls.PROFILES.get(profile, {})
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
        self.build_embeddings()
        self.get_logger().info("Categorization node initialized.")

    def add_entry_callback(self, request, response):
        """Service callback to add items to ChromaDB"""

        try:
            if request.metadata:
                metadatas_ = json.loads(request.metadata)
            else:
                metadatas_ = request.metadata

            # Ensure documents is a list
            documents = (
                request.document
                if isinstance(request.document, list)
                else [request.document]
            )
            metadatas = metadatas_ if metadatas_ else [{} for _ in documents]

            metadata_objects = []

            # Normalize and validate all metadata entries using the profile

            for meta in metadatas:
                try:
                    metadata_parsed = MetadataModel.with_profile(
                        request.collection, **meta
                    )
                    metadata_objects.append(metadata_parsed.model_dump())
                except Exception as e:
                    self.get_logger().error(
                        f"Failed to process metadata entry: {meta} — {str(e)}"
                    )
                    raise

            documents = self.clean(documents)
            # Inject context into documents and preserve original names
            for i, (doc, meta) in enumerate(zip(documents, metadata_objects)):
                meta["original_name"] = doc
                context = meta.get("context")
                if context:
                    documents[i] = f"{doc} {context}"
            # self.get_logger().info(f"This is the request that is reaching{(request.collection, documents, metadata_objects)}")
            # self.get_logger().info("Adding entries to ChromaDB")
            if request.collection == "closest_items":
                self.chroma_adapter._get_or_create_collection("closest_items")
            self.chroma_adapter.add_entries(
                request.collection, documents, metadata_objects
            )

            response.success = True
            response.message = "Item(s) added successfully"
            self.get_logger().info("Add Entry request handled successfully")
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
            if request.collection == "items":
                context = MetadataModel.PROFILES[MetadataProfile.ITEMS]["context"]
            elif request.collection == "locations":
                context = MetadataModel.PROFILES[MetadataProfile.LOCATIONS]["context"]
            elif request.collection == "actions":
                context = MetadataModel.PROFILES[MetadataProfile.ACTIONS]["context"]
            else:
                context = ""

            grouped_results = []
            for query in request.query:
                query_with_context = query + context
                if request.collection == "closest_items":
                    results_raw = self.chroma_adapter.query_where(
                        request.collection, query_with_context
                    )
                else:
                    results_raw = self.chroma_adapter.query(
                        request.collection, [query_with_context], request.topk
                    )
                docs = results_raw.get("documents", [[]])[0]
                metas = results_raw.get("metadatas", [[]])[0]

                formatted_results = []
                for doc, meta in zip(docs, metas):
                    entry = {
                        "document": doc,
                        "metadata": meta if isinstance(meta, dict) else {},
                    }

                    if isinstance(meta, dict) and "original_name" in meta:
                        entry["document"] = meta["original_name"]

                    formatted_results.append(entry)

                grouped_results.append({"query": query, "results": formatted_results})

            response.results = [json.dumps(entry) for entry in grouped_results]

            response.success = bool(grouped_results)
            response.message = (
                "Query successful" if grouped_results else "No matching items found"
            )
            self.get_logger().info("Query request handled successfully")

        except Exception as e:
            response.success = False
            response.message = f"Failed to query items: {str(e)}"
            self.get_logger().error(response.message)
        if request.collection == "closest_items":
            self.chroma_adapter.delete_collection("closest_items")
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
        Reads JSON files from the designated dataframes folder,
        and for each file:
        - Reads documents and (if available) metadata.
        - Gets or creates a corresponding collection.
        - Adds entries to the collection via the add_entries method,
            which will process documents and metadata (adding "original_name",
            appending "context", and cleaning metadata) automatically.
        """
        # Get the directory of the current script
        script_dir = Path(__file__).resolve().parent

        # Define the folder where the JSON files are located
        dataframes_folder = script_dir / "../embeddings/dataframes"
        # Ensure the folder exists
        if not (dataframes_folder.exists() and dataframes_folder.is_dir()):
            raise FileNotFoundError(
                f"The folder {dataframes_folder} does not exist or is not a directory."
            )

        # Get all JSON files in the folder
        dataframes = [
            file.resolve()
            for file in dataframes_folder.iterdir()
            if file.suffix == ".json"
        ]
        # Check if there are any JSON files
        if not dataframes:
            raise FileNotFoundError(
                f"No JSON files found in the folder {dataframes_folder}."
            )
        collections = {}
        documents = []
        metadatas_ = []
        for file in dataframes:
            print("Processing file:", file)
            # Read the JSON file into a Python dictionary
            with open(file, "r") as f:
                data = json.load(f)
            for dict in data:
                document = dict["document"]
                if "metadata" in dict:
                    metadata = dict["metadata"]
                    [document, metadata] = self.add_basics(document, metadata)
                else:
                    metadata = {}
                    [document, metadata] = self.add_basics(document, metadata)
                metadatas_.append(metadata)
                documents.append(dict["document"])

            # Sanitize and get or create the collection
            collection_name = self.chroma_adapter._sanitize_collection_name(file.stem)

            collections[collection_name] = (
                self.chroma_adapter._get_or_create_collection(collection_name)
            )
            print("Collection name:", collection_name)
            self.chroma_adapter.add_entries(collection_name, documents, metadatas_)
        self.chroma_adapter._get_or_create_collection("command_history")
        return

    def add_basics(self, documents, metadatas):
        # Inject context and sanitize document content
        metadatas["original_name"] = documents
        if "context" in metadatas:
            context = metadatas.get("context")
        else:
            context = ""
        documents = f"{documents} {context}" if context else documents

        return documents, metadatas

    def clean(self, documents):
        # If it's a string that looks like a list -> try parsing it
        if (
            isinstance(documents, str)
            and documents.strip().startswith("[")
            and documents.strip().endswith("]")
        ):
            try:
                parsed = json.loads(documents.replace("'", '"'))  # Handle single quotes
                if isinstance(parsed, list):
                    print("document after cleaning:", documents)
                    return " ".join(str(x) for x in parsed)
            except json.JSONDecodeError:
                pass  # Leave it as-is if it fails to parse

        # Default case: just return the string
        return documents


def main():
    rclpy.init()
    embeddings = Embeddings()
    rclpy.spin(embeddings)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
