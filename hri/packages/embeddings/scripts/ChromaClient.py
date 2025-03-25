import chromadb
from chromadb.utils import embedding_functions
import pandas as pd
from pathlib import Path
import json
from uuid import uuid4
from filter import remove_empty_lists, remove_nulls


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
        Method to build embeddings for the household use.
        If a 'context' column is present, the first row is used as the context
        and appended to each entry in the 'name' column.
        """
        # Get the directory of the current script
        script_dir = Path(__file__).resolve().parent
        # Define the folder where the CSV files are located
        dataframes_folder = script_dir / "../embeddings/dataframes"

        # Ensure the folder exists
        if dataframes_folder.exists() and dataframes_folder.is_dir():
            # Get all .csv files in the folder dynamically
            dataframes = [
                file.resolve()
                for file in dataframes_folder.iterdir()
                if file.suffix == ".csv"
            ]
        else:
            raise FileNotFoundError(
                f"The folder {dataframes_folder} does not exist or is not a directory."
            )

        collections = {}
        for file in dataframes:
            print("Processing file:", file)
            # Read the CSV file into a pandas DataFrame
            df = pd.read_csv(file)
            # Check if the csv has a 'document' column
            if "documents" not in df.columns:
                raise ValueError(f"The 'documents' column is missing in {file}")
            documents = df["documents"]
            metadatas_ = []
            if "metadata" not in df.columns:
                print("metadata not detected")
                for doc in df["documents"]:
                    metadatas_.append(
                        {"original_name": doc}
                    )  # Append a new dictionary to the list
            else:
                metadatas_ = self.json2dict(df["metadata"])
                cleaned_metadata = (
                    [remove_empty_lists(remove_nulls(meta)) for meta in metadatas_]
                    if metadatas_
                    else None
                )
                for i, (row, metadata) in enumerate(
                    zip(df["documents"], cleaned_metadata)
                ):
                    metadata["original_name"] = (
                        row  # Add "original_name" to each metadata dictionary
                    )
                    if "context" in metadata.keys():
                        documents[i] = df.at[i, "documents"] + " " + metadata["context"]
                metadatas_ = cleaned_metadata

            # Get or create collection
            collection_name = file.stem
            collection_name = self._sanitize_collection_name(collection_name)
            collections[collection_name] = self._get_or_create_collection(
                collection_name
            )
            # Add entries to the collection
            self.add_entries(collection_name, documents.tolist(), metadatas_)

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

    # def remove_item_by_id(self, collection_name, id):
    #     """Method to remove an item from a collection"""
    #     collection_ = self.get_collection(collection_name)
    #     collection_.delete(ids=[id])
    #     return

    def add_entries(self, collection_name, documents, metadatas=None):
        """Method to add multiple entries with optional metadata"""

        collection_ = self.get_collection(collection_name)
        ids = [str(uuid4()) for _ in range(len(documents))]
        cleaned_metadatas = (
            [remove_nulls(remove_empty_lists(meta)) for meta in metadatas]
            if metadatas
            else None
        )

        # Add documents and metadata to the collection
        return collection_.add(
            ids=ids, documents=documents, metadatas=cleaned_metadatas
        )

    # def get_entry_by_id(self, collection_name, id_):
    #     """Method to get an entry by id"""
    #     collection_ = self.get_collection(collection_name)
    #     return collection_.get(ids=id_, include=["metadatas", "documents"])

    # def get_entry_by_metadata(self, collection_name, metadata):
    #     """Method to get an entry by metadata (input -----> key:value)"""
    #     collection_ = self.get_collection(collection_name)
    #     return collection_.get(where=metadata, include=["metadatas", "documents"])

    # def get_entry_by_document(self, collection_name, document):
    #     """Method to get an entry by document (returns all of the entries that contain the string)"""
    #     collection_ = self.get_collection(collection_name)
    #     return collection_.get(
    #         where_document={"$contains": document}, include=["metadatas", "documents"]
    #     )

    def json2dict(self, json_str):
        """Method to convert a JSON string to a dictionary"""
        metadatas_ = json_str.tolist()
        metadatas_ = [json.loads(meta) for meta in metadatas_]
        metadatas_ = [json.loads(meta) for meta in metadatas_]
        return metadatas_


def main():
    client_ = ChromaClient()
    client_.remove_all_collections()
    client_.build_embeddings()
    # print(client_.list_collections())
    # collection = client_.get_collection("items")
    print(client_.query("items", "apple", 5))
    print(client_.query("locations", "kitchen", 5, "household locations"))
    print(client_.query("actions", "move", 5, "household actions"))
    print(client_.query("categories", "drinks", 5))
    print(client_.query("names", "Luis", 5))
    # print(collection.get())


if __name__ == "__main__":
    main()
