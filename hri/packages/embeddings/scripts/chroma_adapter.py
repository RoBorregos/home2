import json
from datetime import datetime
from uuid import uuid4

import chromadb
from chromadb.utils import embedding_functions
from filter import remove_empty_lists, remove_nulls


class ChromaAdapter:
    def __init__(self):
        self.client = chromadb.HttpClient(host="localhost", port=8000)
        # Configure the embedding function
        self.sentence_transformer_ef = (
            embedding_functions.SentenceTransformerEmbeddingFunction(
                model_name="all-MiniLM-L12-v2"
            )
        )

    def delete_collection(self, collection_name: str):
        """Method to delete a collection"""
        try:
            self.client.delete_collection(name=collection_name)
        except Exception:
            raise ValueError(f"The collection is missing {collection_name}")

    def remove_all_collections(self):
        """Method to remove all collections"""
        collections = self.client.list_collections()
        for collection in collections:
            self.client.delete_collection(name=collection)
        return

    def remove_categorization_collections(self):
        """Method to remove all collections from categorization node"""
        collections = [
            "items",
            "actions",
            "categories",
            "locations",
            "names",
            "command_history",
        ]
        for collection in collections:
            self.client.delete_collection(collection)
        return

    def list_collections(self):
        """Method to list all collections"""
        return self.client.list_collections()

    def get_collection(self, collection_name):
        """Method to get a collection; returns a collection object"""
        try:
            return self.client.get_collection(collection_name)
        except Exception:
            raise ValueError(f"The collection is missing {collection_name}")

    def remove_collection(self, collection_name: str):
        """Method to remove a collection"""
        _ = self.get_collection(collection_name)
        self.client.delete_collection(name=collection_name)
        return

    def _get_or_create_collection(self, collection_name: str):
        """Helper method to get or create a collection"""
        try:
            return self.client.get_or_create_collection(name=collection_name)
        except Exception:
            raise ValueError(f"The collection is missing {collection_name}")

    def query(self, collection_name: str, query, top_k):
        """Method to query the collection and return only the original names from metadata"""
        collection_ = self.get_collection(collection_name)
        results = collection_.query(
            query_texts=query,
            n_results=top_k,
            include=["metadatas", "documents", "distances"],
        )

        return results  # Return only the extracted names

    def query_where(self, collection_name: str, query: str):
        """Method to query the collection and return only the original names from metadata"""
        results = self.get(collection_name, query, "original_name")

        return results

    def _sanitize_collection_name(self, collection):
        """
        Ensures collection name is a valid string due to the constraints of the ChromaDB API.
        The name must be 3 to 63 characters long and only contain alphanumeric characters,
        hyphens, or underscores.
        """
        collection = str(collection).strip()
        if not (
            3 <= len(collection) <= 63
            and collection.replace("-", "").replace("_", "").isalnum()
        ):
            raise ValueError(f"Invalid collection name: {collection}")
        return collection

    def remove_item_by_id(self, collection_name, id):
        """Method to remove an item from a collection"""
        collection_ = self.get_collection(collection_name)
        collection_.delete(ids=[id])
        return

    def add_entries(self, collection_name, documents, metadatas=None):
        """Method to add multiple entries with optional metadata.

        This function processes each document and its associated metadata:
        - Ensures documents is a list.
        - Normalizes metadatas to a list of dictionaries.
        - Cleans metadata with remove_empty_lists and remove_nulls.
        """

        collection_ = self.get_collection(collection_name)

        # Ensure documents is always a list
        if isinstance(documents, str):
            documents = [documents]

        # Ensure metadatas is always a list of dictionaries
        if metadatas is None:
            metadatas = [{} for _ in documents]

        elif isinstance(metadatas, dict):
            metadatas = [metadatas] * len(documents)

        # Clean each metadata dictionary
        cleaned_metadatas = [
            remove_nulls(remove_empty_lists(meta)) for meta in metadatas
        ]
        # Generate a unique id for each document
        ids = [str(uuid4()) for _ in range(len(documents))]
        # Add a timestamp to each metadata dictionary
        for meta in cleaned_metadatas:
            if "timestamp" not in meta:
                meta["timestamp"] = datetime.now().isoformat()
        # Add documents and metadata to the collection
        return collection_.add(
            ids=ids, documents=documents, metadatas=cleaned_metadatas
        )

    def remove_item_by_document(self, collection_name, document):
        """Method to remove an item by document"""
        collection_ = self.get_collection(collection_name)
        return collection_.delete(where={"original_name": document})

    def get_entry_by_document(self, collection_name, document):
        """Method to get an entry by document (returns all of the entries that contain the string)"""
        collection_ = self.get_collection(collection_name)
        return collection_.get(
            where={"original_name": document}, include=["metadatas", "documents"]
        )

    def json2dict(self, json_str):
        """Method to convert a JSON string to a dictionary"""
        metadatas_ = json_str.tolist()
        metadatas_ = [json.loads(meta) for meta in metadatas_]
        metadatas_ = [json.loads(meta) for meta in metadatas_]
        return metadatas_

    def get(self, collection_name: str, document: str, field: str):
        """Method to get an entry by document (returns all of the entries that contain the string)"""
        collection_ = self.get_collection(collection_name)
        return collection_.get(
            where={field: document[0]}, include=["metadatas", "documents"]
        )


def main():
    client_ = ChromaAdapter()
    results = client_.list_collections()
    print(results)


if __name__ == "__main__":
    main()
