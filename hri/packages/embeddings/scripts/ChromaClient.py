import chromadb
from chromadb.utils import embedding_functions
import pandas as pd
from pathlib import Path
import json
from uuid import uuid4


class ChromaClient:
    def __init__(self):
        self.client = chromadb.HttpClient(host="localhost", port=8000)
        # Configure the embedding function
        self.sentence_transformer_ef = (
            embedding_functions.SentenceTransformerEmbeddingFunction(
                model_name="all-MiniLM-L12-v2"
            )
        )

    def list_collections(self):
        """Method to list all collections"""
        return self.client.list_collections()

    def get_collection(self, collection_name):
        """Method to get a collection, it retunrsn a collection object"""
        return self.client.get_collection(collection_name)

    def build_embeddings_callback(self):
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
            # Get the absolute path of the file and create a pandas data frame
            file = file.resolve()
            df = pd.read_csv(file)

            # Check if the csv has a name column
            if "name" not in df.columns:
                raise ValueError(f"The 'name' column is missing in {file}")

            # Add context to each row
            df["name"] = df["name"].apply(lambda x: f"{x} {context}")

            # Check if the collection already exists and has the correct name format
            collection_name = file.stem
            collection_name = self._sanitize_collection_name(collection_name)
            collections[collection_name] = self._get_or_create_collection(
                collection_name
            )

            # There is metadata
            if "metadata" in df.columns:
                # Add the entries in the collection
                collections[collection_name].add(
                    documents=df["name"].tolist(),
                    metadatas=[
                        json.loads(row["metadata"])
                        if "metadata" in row and row["metadata"]
                        else {}
                        for _, row in df.iterrows()
                    ],
                    ids=[str(uuid4()) for _ in range(len(df))],
                )
            # There is no metadata
            else:
                collections[collection_name].add(
                    documents=df["name"].tolist(),
                    ids=[str(uuid4()) for _ in range(len(df))],
                )
        return

    def remove_collection(self, collection_name):
        """Method to remove a collection"""
        try:
            self.client.delete_collection(collection_name)
        except Exception:
            raise ValueError(f"The collection is missing {collection_name}")
        return

    def _get_or_create_collection(self, collection_name):
        """Helper method to get or create a collection"""
        try:
            return self.client.get_or_create_collection(name=collection_name)
        except Exception:
            raise ValueError(f"The collection is missing {collection_name}")

    def get_entry_by_id(self, collection_name, id_):
        """Method to get an entry by id"""
        try:
            collection_ = self.client.get_collection(name=collection_name)
        except Exception:
            raise ValueError(f"The collection is missing {collection_name}")
        return collection_.get(ids=id_, include=["metadatas", "documents"])

    def get_entry_by_metadata(self, collection_name, metadata):
        """Method to get an entry by metadata"""
        try:
            collection_ = self.client.get_collection(name=collection_name)
        except Exception:
            raise ValueError(f"The collection is missing {collection_name}")

        return collection_.get(where=metadata, include=["metadatas", "documents"])

    def get_entry_by_document(self, collection_name, document):
        """Method to get an entry by document"""
        try:
            collection_ = self.client.get_collection(name=collection_name)
        except Exception:
            raise ValueError(f"The collection is missing {collection_name}")

        return collection_.get(
            where_document={"$contains": document}, include=["metadatas", "documents"]
        )

    def add_entry(self, collection_name, document):
        """Method to add an entry to a collection"""
        try:
            collection_ = self.client.get_collection(name=collection_name)
        except Exception:
            raise ValueError(f"The collection is missing {collection_name}")

        return collection_.add(ids=str(uuid4()), documents=[document])

    def add_entry_with_metadata(self, collection_name, document, metadata):
        """Method to add an entry with metadata to a collection"""
        try:
            collection_ = self.client.get_collection(name=collection_name)
        except Exception:
            raise ValueError(f"The collection is missing {collection_name}")

        return collection_.add(
            ids=str(uuid4()), documents=[document], metadatas=[metadata]
        )

    def query(self, collection_name, query, top_k):
        """Method to query the collection"""
        try:
            collection_ = self.client.get_collection(name=collection_name)
        except Exception:
            raise ValueError(f"Does not exist collection: {collection_name}")

        return collection_.query(
            query_texts=query,
            n_results=top_k,
            include=["metadatas", "documents", "distances"],
        )

    def _sanitize_collection_name(self, collection):
        """Ensures collection name is a valid string due to the constraints of the ChromaDB API https://docs.trychroma.com/docs/collections/create-get-delete"""
        collection = str(collection).strip()

        if not (
            3 <= len(collection) <= 63
            and collection.replace("-", "").replace("_", "").isalnum()
        ):
            raise ValueError(f"Invalid collection name: {collection}")

        return collection
