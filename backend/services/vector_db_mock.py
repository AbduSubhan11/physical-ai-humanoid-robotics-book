from typing import List, Dict, Any, Optional
from uuid import UUID
import logging

logger = logging.getLogger(__name__)

class VectorDBService:
    """
    Mock service for handling vector database operations with Qdrant
    This is a simplified version for testing without actual Qdrant connection
    """
    def __init__(self):
        # Mock in-memory storage for testing
        self.mock_collection = {}
        self.collection_name = "book_embeddings"
        logger.info("Initialized mock VectorDBService")

    def initialize_collection(self, vector_size: int = 1536):
        """
        Initialize the collection (mock)
        """
        logger.info(f"Mock initialization of collection {self.collection_name} with vector size {vector_size}")
        self.mock_collection = {}

    def store_embedding(
        self,
        content_id: str,
        chunk_text: str,
        embedding: List[float],
        chunk_index: int = 0,
        metadata: Optional[Dict[str, Any]] = None
    ) -> str:
        """
        Store an embedding in mock storage
        """
        if metadata is None:
            metadata = {}

        # Add content-specific metadata
        metadata.update({
            "content_id": content_id,
            "chunk_text": chunk_text,
            "chunk_index": chunk_index
        })

        # Generate a unique point ID
        point_id = f"{content_id}_{chunk_index}"

        # Store in mock storage
        self.mock_collection[point_id] = {
            "id": point_id,
            "vector": embedding,
            "payload": metadata
        }

        logger.info(f"Mock stored embedding for content {content_id}, chunk {chunk_index}")
        return point_id

    def search_similar(
        self,
        query_embedding: List[float],
        limit: int = 5,
        content_id_filter: Optional[str] = None
    ) -> List[Dict[str, Any]]:
        """
        Search for similar embeddings in mock storage
        This is a simple cosine similarity implementation for testing
        """
        try:
            results = []

            # For testing purposes, return some mock results
            # In a real implementation, this would do actual similarity search
            for point_id, data in self.mock_collection.items():
                if content_id_filter and data["payload"]["content_id"] != content_id_filter:
                    continue

                # Mock similarity score (in a real implementation, calculate actual cosine similarity)
                score = 0.8  # Mock score for testing

                results.append({
                    "id": data["id"],
                    "score": score,
                    "content": data["payload"]["chunk_text"],
                    "content_id": data["payload"]["content_id"],
                    "chunk_index": data["payload"]["chunk_index"],
                    "metadata": {k: v for k, v in data["payload"].items()
                                if k not in ["chunk_text", "content_id", "chunk_index"]}
                })

            # Sort by score (mock) and return top results
            results = sorted(results, key=lambda x: x["score"], reverse=True)[:limit]

            logger.info(f"Mock found {len(results)} similar embeddings")
            return results
        except Exception as e:
            logger.error(f"Failed to search similar embeddings: {e}")
            return []

    def delete_embeddings(self, content_id: str):
        """
        Delete all embeddings for a specific content ID
        """
        try:
            keys_to_delete = []
            for point_id, data in self.mock_collection.items():
                if data["payload"]["content_id"] == content_id:
                    keys_to_delete.append(point_id)

            for key in keys_to_delete:
                del self.mock_collection[key]

            logger.info(f"Mock deleted embeddings for content {content_id}")
        except Exception as e:
            logger.error(f"Failed to delete embeddings: {e}")

    def get_embedding_count(self) -> int:
        """
        Get the total number of embeddings in the collection
        """
        count = len(self.mock_collection)
        logger.info(f"Mock total embeddings in collection: {count}")
        return count

    def health_check(self) -> bool:
        """
        Check if the mock service is healthy
        """
        return True