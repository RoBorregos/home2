#!/usr/bin/env python3
from pathlib import Path
import pandas as pd
import chromadb
from chromadb.utils import embedding_functions
import rclpy
from rclpy.node import Node
from frida_interfaces.srv import (
    AddItem,
    RemoveItem,
    UpdateItem,
    QueryItem,
)  # Updated service imports

# Test Commands:
# ros2 service call /query_item frida_interfaces/srv/QueryItem "{query: 'kitchen', collection: 'locations', topk: 2}"
# ros2 service call /update_item frida_interfaces/srv/UpdateItem "{item_id: ['locations_1'], field: ['document'], new_data: ['Updated kitchen item'], collection: 'locations'}"
# ros2 service call /add_item frida_interfaces/srv/AddItem "{document: ['Test Item'], id: ['test_id'], collection: 'items'}"
# ros2 service call /remove_item frida_interfaces/srv/RemoveItem "{item_id: ['test_id'], collection: 'items'}"


class Embeddings(Node):
    def __init__(self):
        super().__init__("embeddings")

        # Initialize services
        self.add_item_service = self.create_service(
            AddItem, "add_item", self.add_item_callback
        )
        self.remove_item_service = self.create_service(
            RemoveItem, "remove_item", self.remove_item_callback
        )
        self.update_item_service = self.create_service(
            UpdateItem, "update_item", self.update_item_callback
        )
        self.query_item_service = self.create_service(
            QueryItem, "query_item", self.query_item_callback
        )

        # Initialize ChromaDB client
        script_dir = Path(__file__).resolve().parent
        chroma_path = str(script_dir / "../chromadb")
        self.chroma_client = chromadb.PersistentClient(path=chroma_path)

        # Configure the embedding function
        self.sentence_transformer_ef = (
            embedding_functions.SentenceTransformerEmbeddingFunction(
                model_name="all-MiniLM-L12-v2"
            )
        )

    def add_item_callback(self, request, response):
        """Service callback to add items to ChromaDB"""
        try:
            collection_name = self._sanitize_collection_name(request.collection)
            collection = self._get_or_create_collection(collection_name)

            collection.add(
                documents=request.document,
                metadatas=[{"text": doc} for doc in request.document],
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

            results = collection.query(
                query_texts=[request.query], n_results=request.topk
            )

            # Ensure results exist before accessing
            response.results = [str(doc) for doc in results.get("documents", [])]
            response.success = True if response.results else False
            response.message = (
                "Query successful" if response.results else "No matching items found"
            )
        except Exception as e:
            response.success = False
            response.message = f"Failed to query items: {str(e)}"
            self.get_logger().error(response.message)
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

    def build_embeddings(self):
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


def main():
    rclpy.init()
    embeddings = Embeddings()
    embeddings.build_embeddings()  # Call the embedding builder if needed at startup
    rclpy.spin(embeddings)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
