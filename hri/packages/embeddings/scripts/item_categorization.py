#!/usr/bin/env python3
from pathlib import Path
import json
import chromadb
import pandas as pd
import rclpy
from chromadb.utils import embedding_functions
from rclpy.node import Node
from rclpy.parameter import Parameter
from pydantic import BaseModel, ValidationError
from typing import Optional
from frida_interfaces.srv import (
    AddItem,
    BuildEmbeddings,
    QueryItem,
    RemoveItem,
    UpdateItem,
)
"""JSON parsing validation model for metadata"""
class MetadataModel(BaseModel):
    shelve: Optional[str] 
    category: Optional[str] 
    timestamp: Optional[str]
    #TODOother metadata fields wil be defined here


class Embeddings(Node):
    def __init__(self):
        super().__init__("embeddings")
        self.get_logger().info("Initializing item_categorization.")
        # Declare parameters for the sentence transformer model and collections built flag
        self.declare_parameter("Embeddings_model", "all-MiniLM-L12-v2")
        self.declare_parameter("collections_built", 0)  # Default: 0 (not built)

        # Parameters for services
        self.declare_parameter("ADD_ITEM_SERVICE", "add_item")
        self.declare_parameter("REMOVE_ITEM_SERVICE", "remove_item")
        self.declare_parameter("UPDATE_ITEM_SERVICE", "update_item")
        self.declare_parameter("QUERY_ITEM_SERVICE", "query_item")
        self.declare_parameter("BUILD_EMBEDDINGS_SERVICE", "build_embeddings")

        # Resolve parameters
        model_name_ = (
            self.get_parameter("Embeddings_model").get_parameter_value().string_value
        )
        add_item_service = (
            self.get_parameter("ADD_ITEM_SERVICE").get_parameter_value().string_value
        )
        remove_item_service = (
            self.get_parameter("REMOVE_ITEM_SERVICE").get_parameter_value().string_value
        )
        update_item_service = (
            self.get_parameter("UPDATE_ITEM_SERVICE").get_parameter_value().string_value
        )
        query_item_service = (
            self.get_parameter("QUERY_ITEM_SERVICE").get_parameter_value().string_value
        )
        build_embeddings_service = (
            self.get_parameter("BUILD_EMBEDDINGS_SERVICE")
            .get_parameter_value()
            .string_value
        )

        # Create the BuildEmbeddings service
        self.build_embeddings_service = self.create_service(
            BuildEmbeddings, build_embeddings_service, self.build_embeddings_callback
        )
        # Initialize ChromaDB client
        self.chroma_client = chromadb.HttpClient(host="localhost", port=8000)
        # Configure the embedding function
        self.sentence_transformer_ef = (
            embedding_functions.SentenceTransformerEmbeddingFunction(
                model_name=model_name_
            )
        )

        # Check if collections are built or need to be built
        self.check_and_update_collections()

        # Initialize services
        self.add_item_service = self.create_service(
            AddItem, add_item_service, self.add_item_callback
        )

        self.remove_item_service = self.create_service(
            RemoveItem, remove_item_service, self.remove_item_callback
        )

        self.update_item_service = self.create_service(
            UpdateItem, update_item_service, self.update_item_callback
        )
        self.query_item_service = self.create_service(
            QueryItem, query_item_service, self.query_item_callback
        )

        self.get_logger().info("item_categorization initialized.")

    def check_and_update_collections(self):
        """Check if collections exist and call the method to build them if missing."""
        collections = [
            "items",
            "locations",
            "categories",
            "actions",
        ]  # List of expected collections
        collections_built = 1  # Assume collections are built unless proven otherwise

        for collection_name in collections:
            try:
                # Check if the collection exists
                collection = self.chroma_client.get_collection(name=collection_name)
                if collection is None:
                    collections_built = (
                        0  # If any collection is missing, set to 0 (not built)
                    )
                    break  # No need to check further
            except Exception:
                collections_built = (
                    0  # If any error occurs (collection doesn't exist), set to 0
                )
                break  # No need to check further

        # Update the ROS parameter based on whether collections are built
        self.set_parameters([Parameter("collections_built", value=collections_built)])
        if collections_built:
            self.get_logger().info("Collections already built, skipping build.")
        else:
            self.get_logger().info(
                "Collections not found, proceeding to build collections."
            )
            self.build_embeddings_callback()  # Build the collections if not built

    def add_item_callback(self, request, response):
        """Service callback to add items to ChromaDB"""
        try:
            try:
                #check that the metadata has the same format as required
                metadata_parsed = MetadataModel.model_validate_json(request.metadata)
                metadata_parsed = metadata_parsed.model_dump()
            except ValidationError as e:
                response.success = False
                response.message = f"Invalid metadata: {str(e)}"
                return response
            
            collection_name = self._sanitize_collection_name(request.collection)
            collection = self._get_or_create_collection(collection_name)

            collection.add(
                documents=request.document,
                metadatas=[metadata_parsed],
                ids=request.id,
            )
            response.success = True
            response.message = "Items added successfully"
        except Exception as e:
            response.success = False
            response.message = f"Failed to add items: {str(e)}"
            self.get_logger().error(response.message)
        return response

    def remove_item_callback(self, request, response):
        """Service callback to remove items from ChromaDB"""
        try:
            collection_name = self._sanitize_collection_name(request.collection)
            collection = self._get_or_create_collection(collection_name)

            self.get_logger().info(
                f"Removing items: {request.item_id} from collection: {collection_name}"
            )
            collection.delete(ids=request.item_id)

            response.success = True
            response.message = "Items removed successfully"
        except Exception as e:
            response.success = False
            response.message = f"Failed to remove items: {str(e)}"
            self.get_logger().error(response.message)
        return response

    def update_item_callback(self, request, response):
        """Service callback to update items in ChromaDB"""
        try:
            collection_name = self._sanitize_collection_name(request.collection)
            collection = self._get_or_create_collection(collection_name)

            for item_id, field, new_data in zip(
                request.item_id, request.field, request.new_data
            ):
                self.get_logger().info(
                    f"Updating {field} for item_id={item_id} with new_data={new_data}"
                )

                update_args = {"ids": [item_id]}
                if field == "document":
                    update_args["documents"] = [new_data]
                elif field == "metadata":
                    update_args["metadatas"] = [{"text": new_data}]

                collection.update(**update_args)

            response.success = True
            response.message = "Items updated successfully"
        except Exception as e:
            response.success = False
            response.message = f"Failed to update items: {str(e)}"
            self.get_logger().error(response.message)
        return response

    def query_item_callback(self, request, response):
        """Service callback to query items from ChromaDB"""
        try:
            collection_name = self._sanitize_collection_name(request.collection)
            collection = self._get_or_create_collection(collection_name)
            where_condition = {}
            if request.where:
                try:
                    where_condition = json.loads(
                        request.where
                    )  # Convert stringified JSON to a dictionary
                    self.get_logger().info(f"Parsed where condition: {where_condition}")
                    results = collection.query(
                        query_texts=[request.query],
                        n_results=request.topk,
                        where=where_condition,
                        include=["documents", "metadatas", "distances"],
                    )
                except json.JSONDecodeError:
                    response.success = False
                    response.message = "Invalid JSON format in where condition."
                    return response
            else:
                self.get_logger().info(
                    "No where condition provided, using empty condition."
                )
                results = collection.query(
                    query_texts=[request.query],
                    n_results=request.topk,
                    include=["documents", "metadatas", "distances"],
                )

            self.get_logger().info(f"Raw query results: {results}")

            # Ensure results exist before accessing
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
        script_dir = Path(__file__).resolve().parent
        dataframes = [
            (
                script_dir / "../embeddings/dataframes/items.csv",
                "Household items context",
            ),
            (
                script_dir / "../embeddings/dataframes/locations.csv",
                "Household locations context",
            ),
            (
                script_dir / "../embeddings/dataframes/categories.csv",
                "Household categories context",
            ),
            (
                script_dir / "../embeddings/dataframes/actions.csv",
                "Household actions context",
            ),
        ]

        collections = {}
        for file, context in dataframes:
            file = file.resolve()  # Ensure the path is absolute
            df = pd.read_csv(file)

            if "name" not in df.columns:
                raise ValueError(f"The 'name' column is missing in {file}")

            df["name"] = df["name"].apply(lambda x: f"{x} {context}")
            collection_name = file.stem
            collections[collection_name] = self._get_or_create_collection(
                collection_name
            )

            collections[collection_name].add(
                documents=df["name"].tolist(),
                metadatas=[
                    {
                        "text": row["name"],
                        "original_name": row["name"].split(" ")[0],
                        **row.to_dict(),
                    }
                    for _, row in df.iterrows()
                ],
                ids=[f"{collection_name}_{i}" for i in range(len(df))],
            )

        self.get_logger().info("Build request received and handled successfully")
        response.success = True
        response.message = "Embeddings built successfully"
        return response

    def _sanitize_collection_name(self, collection):
        """Ensures collection name is a valid string"""
        if isinstance(collection, list):
            collection = collection[0]  # Extract from list if necessary

        collection = str(collection).strip()

        if not (
            3 <= len(collection) <= 63
            and collection.replace("-", "").replace("_", "").isalnum()
        ):
            self.get_logger().error(f"Invalid collection name: {collection}")
            raise ValueError(f"Invalid collection name: {collection}")

        return collection

    def _get_or_create_collection(self, collection_name):
        """Helper method to get or create a collection"""
        try:
            return self.chroma_client.get_or_create_collection(name=collection_name)
        except Exception as e:
            self.get_logger().error(
                f"Error retrieving collection '{collection_name}': {str(e)}"
            )
            raise


def main():
    rclpy.init()
    embeddings = Embeddings()
    rclpy.spin(embeddings)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
