#!/usr/bin/env python3
import os
from pathlib import Path
import pandas as pd
from sentence_transformers import SentenceTransformer
import chromadb
from chromadb.utils import embedding_functions
import rclpy
from rclpy.node import Node
from frida_interfaces.srv import AddItem, RemoveItem, UpdateItem, QueryItem  # Updated service imports


class Embeddings(Node):
    def __init__(self):
        super().__init__('embeddings')

        # Initialize services
        self.add_item_service = self.create_service(AddItem, 'add_item', self.add_item_callback)
        self.remove_item_service = self.create_service(RemoveItem, 'remove_item', self.remove_item_callback)
        self.update_item_service = self.create_service(UpdateItem, 'update_item', self.update_item_callback)
        self.query_item_service = self.create_service(QueryItem, 'query_item', self.query_item_callback)

        # Initialize ChromaDB client
        script_dir = Path(__file__).resolve().parent
        chroma_path = str(script_dir / "../chromadb")
        self.chroma_client = chromadb.PersistentClient(path=chroma_path)

        # Configure the embedding function
        self.sentence_transformer_ef = embedding_functions.SentenceTransformerEmbeddingFunction(
            model_name="all-MiniLM-L12-v2"
        )

    def add_item_callback(self, request, response):
        """Service callback to add items to ChromaDB"""
        try:
            collection = self._get_or_create_collection(request.collection)
            collection.add(
                documents=request.document,
                metadatas=[{'text': doc} for doc in request.document],
                ids=request.id
            )
            response.success = True
            response.message = "Items added successfully"
        except Exception as e:
            response.success = False
            response.message = f"Failed to add items: {str(e)}"
        return response

    def remove_item_callback(self, request, response):
        """Service callback to remove items from ChromaDB"""
        try:
            collection = self._get_or_create_collection(request.collection)
            collection.delete(ids=request.item_id)
            response.success = True
            response.message = "Items removed successfully"
        except Exception as e:
            response.success = False
            response.message = f"Failed to remove items: {str(e)}"
        return response

    def update_item_callback(self, request, response):
        """Service callback to update items in ChromaDB"""
        try:
            collection = self._get_or_create_collection(request.collection)
            for item_id, field, new_data in zip(request.item_id, request.field, request.new_data):
                # Assuming the field is the 'document' or 'metadata' field that needs updating
                collection.update(
                    ids=[item_id],
                    documents=[new_data] if field == 'document' else None,
                    metadatas=[{'text': new_data}] if field == 'metadata' else None
                )
            response.success = True
            response.message = "Items updated successfully"
        except Exception as e:
            response.success = False
            response.message = f"Failed to update items: {str(e)}"
        return response

    def query_item_callback(self, request, response):
        """Service callback to query items from ChromaDB"""
        try:
            collection = self._get_or_create_collection(request.collection)
            results = collection.query(
                query_texts=[request.query],
                n_results=request.topk
            )
            response.results = [str(doc) for doc in results['documents']]
            response.success = True
            response.message = "Query successful"
        except Exception as e:
            response.success = False
            response.message = f"Failed to query items: {str(e)}"
        return response

    def _get_or_create_collection(self, collection_name):
        """Helper method to get or create a collection"""
        collection = self.chroma_client.get_or_create_collection(name=collection_name)
        return collection

    def build_embeddings(self):
        """Method to build embeddings for the household items data"""
        script_dir = Path(__file__).resolve().parent
        dataframes = [
            (script_dir / "../embeddings/dataframes/items.csv", "Household items context"),
            (script_dir / "../embeddings/dataframes/locations.csv", "Household locations context"),
            (script_dir / "../embeddings/dataframes/categories.csv", "Household categories context"),
            (script_dir / "../embeddings/dataframes/actions.csv", "Household actions context"),
        ]
        
        # Build the collections from CSV files
        collections = {}
        for file, context in dataframes:
            file = file.resolve()  # Ensure the path is absolute
            
            df = pd.read_csv(file)
            
            if 'name' not in df.columns:
                raise ValueError(f"The 'name' column is missing in {file}")
            
            df['name'] = df['name'].apply(lambda x: f"{x} {context}")
            
            collection_name = file.stem
            collections[collection_name] = self._get_or_create_collection(collection_name)
            
            collections[collection_name].add(
                documents=df['name'].tolist(),
                metadatas=[{
                    'text': row['name'],
                    'original_name': row['name'].split(" ")[0],
                    **row.to_dict()
                } for _, row in df.iterrows()],
                ids=[f"{collection_name}_{i}" for i in range(len(df))]
            )
        
        self.get_logger().info('Build request received and handled successfully')


def main():
    rclpy.init()
    embeddings = Embeddings()
    embeddings.build_embeddings()  # Call the embedding builder if needed at startup
    rclpy.spin(embeddings)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
