#!/usr/bin/env python3

"""
ROS 2 node that exposes PostgresAdapter operations as ROS services.
This allows the integration container to call HRI postgres operations
without directly importing the embeddings package.
"""

import json

import numpy as np
import rclpy
from embeddings.postgres_adapter import PostgresAdapter
from frida_constants.hri_constants import (
    ADD_ENTRY_SERVICE,
    FIND_CLOSEST_SERVICE,
    QUERY_ENTRY_SERVICE,
)
from frida_interfaces.srv import AddEntry, FindClosest, QueryEntry
from rclpy.node import Node


class PostgresService(Node):
    def __init__(self):
        super().__init__("postgres_service")
        self.get_logger().info("Initializing PostgresService node...")

        self.pg = PostgresAdapter()

        self.add_entry_srv = self.create_service(
            AddEntry, ADD_ENTRY_SERVICE, self.add_entry_callback
        )
        self.query_entry_srv = self.create_service(
            QueryEntry, QUERY_ENTRY_SERVICE, self.query_entry_callback
        )
        self.find_closest_srv = self.create_service(
            FindClosest, FIND_CLOSEST_SERVICE, self.find_closest_callback
        )

        self.get_logger().info("PostgresService node ready.")

    def add_entry_callback(self, request, response):
        try:
            collection = request.collection
            metadata = json.loads(request.metadata) if request.metadata else {}
            document = list(request.document)

            if collection == "command_history":
                command = document[0] if document else ""
                self.pg.add_command(
                    action=metadata.get("action", ""),
                    command=command,
                    result=metadata.get("result", ""),
                    status=metadata.get("status", ""),
                    context=metadata.get("context", ""),
                )
            elif collection == "items":
                text = document[0] if document else ""
                context = metadata.get("context", "")
                self.pg.add_item2(text=text, context=context)
            elif collection == "locations":
                area = metadata.get("area", document[0] if document else "")
                subarea = metadata.get("subarea", "")
                self.pg.add_location2(area=area, subarea=subarea)

            response.success = True
            response.message = f"Entry added to '{collection}'"
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"AddEntry error [{request.collection}]: {e}")

        return response

    def query_entry_callback(self, request, response):
        try:
            collection = request.collection
            metadata = json.loads(request.metadata) if request.metadata else {}
            query = request.query[0] if request.query else ""
            top_k = request.topk if request.topk > 0 else 10

            if collection == "locations":
                use_context = metadata.get("use_context", False)
                results = self.pg.query_location(
                    query, top_k=top_k, use_context=use_context
                )
                response.results = [
                    json.dumps(
                        {
                            "id": r.id,
                            "area": r.area,
                            "subarea": r.subarea,
                            "context": r.context or "",
                            "similarity": r.similarity,
                        }
                    )
                    for r in results
                ]

            elif collection == "items":
                items = self.pg.get_all_items()
                response.results = [
                    json.dumps(
                        {
                            "id": item.id,
                            "text": item.text,
                            "context": item.context or "",
                        }
                    )
                    for item in items
                ]

            elif collection == "command_history":
                action = metadata.get("action", None)
                results = self.pg.query_command_history(
                    command=query, action=action, top_k=top_k
                )
                response.results = [
                    json.dumps(
                        {
                            "id": r.id,
                            "action": r.action,
                            "command": r.command,
                            "result": r.result,
                            "status": r.status,
                            "similarity": r.similarity,
                            "context": r.context or "",
                        }
                    )
                    for r in results
                ]

            elif collection == "hand_items":
                rows_by_name, rows_by_description = self.pg.get_hand_items(query)
                response.results = [
                    json.dumps(
                        [
                            {
                                "id": r.id,
                                "name": r.name,
                                "description": r.description,
                                "x_loc": r.x_loc,
                                "y_loc": r.y_loc,
                                "m_loc_x": r.m_loc_x,
                                "m_loc_y": r.m_loc_y,
                                "color": r.color,
                            }
                            for r in rows_by_name
                        ]
                    ),
                    json.dumps(
                        [
                            {
                                "id": r.id,
                                "name": r.name,
                                "description": r.description,
                                "x_loc": r.x_loc,
                                "y_loc": r.y_loc,
                                "m_loc_x": r.m_loc_x,
                                "m_loc_y": r.m_loc_y,
                                "color": r.color,
                            }
                            for r in rows_by_description
                        ]
                    ),
                ]

            response.success = True
            response.message = f"Query on '{collection}' successful"
        except Exception as e:
            response.success = False
            response.message = str(e)
            response.results = []
            self.get_logger().error(f"QueryEntry error [{request.collection}]: {e}")

        return response

    def find_closest_callback(self, request, response):
        try:
            documents = list(request.documents)
            query = request.query
            top_k = request.top_k if request.top_k > 0 else 1
            threshold = request.threshold

            if not documents:
                response.success = False
                response.message = "No documents provided"
                response.results = []
                response.similarities = []
                return response

            doc_embeddings = self.pg.embedding_model.encode(
                documents, convert_to_tensor=False
            )
            query_embedding = self.pg.embedding_model.encode(
                query, convert_to_tensor=False
            )

            def cos_sim(x, y):
                return float(np.dot(x, y) / (np.linalg.norm(x) * np.linalg.norm(y)))

            scored = [
                (doc, cos_sim(emb, query_embedding))
                for doc, emb in zip(documents, doc_embeddings)
            ]
            scored = [(doc, sim) for doc, sim in scored if sim >= threshold]
            scored = sorted(scored, key=lambda x: x[1], reverse=True)[:top_k]

            response.results = [doc for doc, _ in scored]
            response.similarities = [sim for _, sim in scored]
            response.success = True
            response.message = "FindClosest successful"
        except Exception as e:
            response.success = False
            response.message = str(e)
            response.results = []
            response.similarities = []
            self.get_logger().error(f"FindClosest error: {e}")

        return response


def main(args=None):
    rclpy.init(args=args)
    node = PostgresService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
