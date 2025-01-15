#!/usr/bin/env python3
"""
ROS2 node that returns the best match of an item or list of items
"""

import rclpy
import os
import pickle
from sentence_transformers import SentenceTransformer, util
from frida_hri_interfaces.srv import ItemsCategory
from ament_index_python.packages import get_package_share_directory


DATAFRAMES_DIR = os.path.join(
    get_package_share_directory("frida_language_processing"), "scripts/dataframes/"
)
# Minimum similarity percentage in which 2 words are considered the same
SIMILARITY_THRESHOLD = 0.7

ITEMS_EXTRACT_TOPIC = "items_category"
OBJECT_EXTRACT_TOPIC = "object_category"


class ItemCategorization:
    """class to handle both services for item categorization"""

    def __init__(self) -> None:
        """Initialize the ROS node"""
        self._node = rclpy.create_node("item_categorization")
        # services
        # self._node.create_service(ItemsCategory,ITEMS_EXTRACT_TOPIC, self.get_local_embedding)
        self._node.create_service(
            ItemsCategory, OBJECT_EXTRACT_TOPIC, self.extract_item
        )
        # load the model
        self.model = SentenceTransformer("all-MiniLM-L12-v2")
        current_dir = "/home-hri/nlp/Embeddings analysis/dataframes"
        # Load the embeddings already calculated/ embeddings database
        with open(os.path.join(current_dir, "embeddings_items.pkl"), "rb") as f:
            self.items_dataframe = pickle.load(f)

        with open(os.path.join(current_dir, "embeddings_categories.pkl"), "rb") as f:
            self.category_embeddings = pickle.load(f)

        with open(os.path.join(current_dir, "embeddings_locations.pkl"), "rb") as f:
            self.locations_embeddings = pickle.load(f)

        with open(os.path.join(current_dir, "embeddings_actions.pkl"), "rb") as f:
            self.actions_embeddings = pickle.load(f)
            # spin the node
        rclpy.spin(self._node)

    def extract_item(self, req):
        """Service to return the best match of the items given"""
        response = ItemsCategory.Response()
        i = 0
        if req.category == "items":
            for item in req.items:
                item_embedding = [self.get_local_embedding(item)]
                possible_items = self.get_similar_items(
                    item_embedding[0], self.items_dataframe
                )
                response.matches[i] = possible_items.iloc[0]["name"]
                i = i + 1
        elif req.category == "locations":
            for location in req.items:
                location_embedding = [self.get_local_embedding(location)]
                possible_locations = self.get_similar_items(
                    location_embedding[0], self.locations_embeddings
                )
                response.matches[i] = possible_locations.iloc[0]["name"]
                i = i + 1
        elif req.category == "actions":
            for action in req.items:
                action_embedding = [self.get_local_embedding(action)]
                possible_actions = self.get_similar_items(
                    action_embedding[0], self.actions_embeddings
                )
                response.matches[i] = possible_actions.iloc[0]["name"]
                i = i + 1
        elif req.category == "category":
            for category in req.items:
                category_embedding = [self.get_local_embedding(category)]
                possible_categories = self.get_similar_items(
                    category_embedding[0], self.category_embeddings
                )
                response.matches[i] = possible_categories.iloc[0]["name"]
                i = i + 1
        return response

    def get_local_embedding(self, object):
        embedding = self.model.encode(object, convert_to_tensor=True)
        return [embedding]

    def get_embeddings_similarity(self, target, category):
        return util.pytorch_cos_sim(target, category).item()

    def get_similar_items(self, input, dataframe):
        """Get the category of the item based on the similarity of the embedding"""
        # Get name similarity
        dataframe["similarity"] = dataframe["name_embedding"].apply(
            lambda x: util.pytorch_cos_sim(input, x).item()
        )
        sorted_names = dataframe.sort_values("similarity", ascending=False)
        return sorted_names


if __name__ == "__main__":
    try:
        ItemCategorization()
    except rclpy.ROSInterruptException:
        pass
