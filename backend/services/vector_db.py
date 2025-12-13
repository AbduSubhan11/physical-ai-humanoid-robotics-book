from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from ..config import settings
from uuid import UUID
import logging

logger = logging.getLogger(__name__)

class VectorDBService:
    """
    Service for handling vector database operations with Qdrant
    """
    def __init__(self):
        # Initialize Qdrant client
        if settings.QDRANT_API_KEY:
            self.client = QdrantClient(
                url=settings.QDRANT_URL,
                api_key=settings.QDRANT_API_KEY,
                prefer_grpc=False  # Using HTTP for simplicity
            )
        else:
            self.client = QdrantClient(url=settings.QDRANT_URL)

        # Collection name for storing embeddings
        self.collection_name = "book_embeddings"

    def initialize_collection(self, vector_size: int = 1536):
        """
        Initialize the collection in Qdrant with specified vector size
        """
        try:
            # Check if collection already exists
            self.client.get_collection(self.collection_name)
            logger.info(f"Collection {self.collection_name} already exists")
        except:
            # Create new collection with cosine distance
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=vector_size,
                    distance=models.Distance.COSINE
                )
            )
            logger.info(f"Created collection {self.collection_name} with vector size {vector_size}")

    def store_embedding(
        self,
        content_id: str,
        chunk_text: str,
        embedding: List[float],
        chunk_index: int = 0,
        metadata: Optional[Dict[str, Any]] = None
    ) -> str:
        """
        Store an embedding in Qdrant
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

        try:
            # Upsert the embedding
            self.client.upsert(
                collection_name=self.collection_name,
                points=[
                    models.PointStruct(
                        id=point_id,
                        vector=embedding,
                        payload=metadata
                    )
                ]
            )
            logger.info(f"Stored embedding for content {content_id}, chunk {chunk_index}")
            return point_id
        except Exception as e:
            logger.error(f"Failed to store embedding: {e}")
            raise

    def search_similar(
        self,
        query_embedding: List[float],
        limit: int = 5,
        content_id_filter: Optional[str] = None
    ) -> List[Dict[str, Any]]:
        """
        Search for similar embeddings in Qdrant
        """
        try:
            # Prepare filters if content_id is specified
            filters = None
            if content_id_filter:
                filters = models.Filter(
                    must=[
                        models.FieldCondition(
                            key="content_id",
                            match=models.MatchValue(value=content_id_filter)
                        )
                    ]
                )

            # Perform search
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=limit,
                query_filter=filters,
                with_payload=True,
                with_vectors=False
            )

            # Format results
            results = []
            for result in search_results:
                results.append({
                    "id": result.id,
                    "score": result.score,
                    "content": result.payload.get("chunk_text", ""),
                    "content_id": result.payload.get("content_id", ""),
                    "chunk_index": result.payload.get("chunk_index", 0),
                    "metadata": {k: v for k, v in result.payload.items()
                                if k not in ["chunk_text", "content_id", "chunk_index"]}
                })

            logger.info(f"Found {len(results)} similar embeddings")
            return results
        except Exception as e:
            logger.error(f"Failed to search similar embeddings: {e}")
            raise

    def delete_embeddings(self, content_id: str):
        """
        Delete all embeddings for a specific content ID
        """
        try:
            # Find all points with the content_id
            scroll_results = self.client.scroll(
                collection_name=self.collection_name,
                scroll_filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="content_id",
                            match=models.MatchValue(value=content_id)
                        )
                    ]
                ),
                limit=10000  # Adjust based on expected max embeddings per content
            )

            # Extract point IDs
            if scroll_results[0]:  # Check if any results were returned
                point_ids = [point.id for point in scroll_results[0]]

                if point_ids:
                    # Delete the points
                    self.client.delete(
                        collection_name=self.collection_name,
                        points_selector=models.PointIdsList(
                            points=point_ids
                        )
                    )
                    logger.info(f"Deleted {len(point_ids)} embeddings for content {content_id}")
                else:
                    logger.info(f"No embeddings found for content {content_id}")
            else:
                logger.info(f"No embeddings found for content {content_id}")
        except Exception as e:
            logger.error(f"Failed to delete embeddings: {e}")
            raise

    def get_embedding_count(self) -> int:
        """
        Get the total number of embeddings in the collection
        """
        try:
            collection_info = self.client.get_collection(self.collection_name)
            count = collection_info.points_count
            logger.info(f"Total embeddings in collection: {count}")
            return count
        except Exception as e:
            logger.error(f"Failed to get embedding count: {e}")
            raise

    def health_check(self) -> bool:
        """
        Check if the Qdrant connection is healthy
        """
        try:
            # Try to get collection info to verify connection
            self.client.get_collection(self.collection_name)
            return True
        except:
            try:
                # If collection doesn't exist, try to create it (also tests connection)
                self.initialize_collection()
                return True
            except:
                return False