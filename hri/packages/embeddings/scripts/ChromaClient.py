import chromadb
from chromadb.utils import embedding_functions
import pandas as pd
from pathlib import Path
import json
from uuid import uuid4
from filter import remove_empty_lists, remove_nulls
from datetime import datetime


class ChromaClient:
    def __init__(self):
        self.client = chromadb.HttpClient(host="localhost", port=8000)
        # Configure the embedding function
        self.sentence_transformer_ef = (
            embedding_functions.SentenceTransformerEmbeddingFunction(
                model_name="all-MiniLM-L12-v2"
            )
        )

    def remove_all_collections(self):
        """Method to remove all collections"""
        collections = self.client.list_collections()
        for collection in collections:
            self.client.delete_collection(name=collection)
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
                metadatas_ = self.json2dict(df["metadata"])

            # Sanitize and get or create the collection
            collection_name = self._sanitize_collection_name(file.stem)
            collections[collection_name] = self._get_or_create_collection(
                collection_name
            )

            self.add_entries(collection_name, documents, metadatas_)

        return

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

    def query(self, collection_name: str, query, top_k, context: str = ""):
        """Method to query the collection and return only the original names from metadata"""
        collection_ = self.get_collection(collection_name)
        results = collection_.query(
            query_texts=query + context,
            n_results=top_k,
            include=["metadatas", "documents", "distances"],
        )
        if results["metadatas"]:  # Ensure "metadatas" is not empty
            results = [
                meta.get("original_name", doc)
                for meta, doc in zip(results["metadatas"][0], results["documents"][0])
            ]
        else:
            results = results["documents"][
                0
            ]  # Access first (and only) list inside "documents"

        return results  # Return only the extracted names

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


def main():
    client_ = ChromaClient()
    # client_.remove_all_collections()
    # client_.build_embeddings()
    # print(client_.list_collections())
    collection = client_.get_collection("items")
    # print(client_.query("items", "apple", 5))
    # print(client_.query("locations", "kitchen", 5, "household locations"))
    # print(client_.query("actions", "move", 5, "household actions"))
    # print(client_.query("categories", "drinks", 5))
    # print(client_.query("names", "Luis", 5))
    print(collection.get())
    # client_.add_entries("items", "chocomilk", {"context": "household items"})
    # print(client_.get_entry_by_document("items", "chocomilk"))


if __name__ == "__main__":
    main()
