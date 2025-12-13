import numpy as np
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from ..config import settings

class EmbeddingManager:
    """
    Manager for handling embeddings in Qdrant
    """
    def __init__(self):
        # Initialize Qdrant client
        if settings.QDRANT_API_KEY:
            self.client = QdrantClient(
                url=settings.QDRANT_URL,
                api_key=settings.QDRANT_API_KEY
            )
        else:
            self.client = QdrantClient(url=settings.QDRANT_URL)

        # Collection name for storing embeddings
        self.collection_name = "book_embeddings"

    def create_collection(self, vector_size: int = 1536):
        """
        Create a collection in Qdrant for storing embeddings
        """
        try:
            # Check if collection already exists
            self.client.get_collection(self.collection_name)
            print(f"Collection {self.collection_name} already exists")
        except:
            # Create new collection
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=vector_size,
                    distance=models.Distance.COSINE
                )
            )
            print(f"Created collection {self.collection_name}")

    def store_embedding(self,
                       content_id: str,
                       chunk_text: str,
                       embedding: List[float],
                       chunk_index: int = 0,
                       metadata: Optional[Dict[str, Any]] = None) -> str:
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

        # Upsert the embedding
        point_id = f"{content_id}_{chunk_index}"
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

        return point_id

    def search_similar(self,
                      query_embedding: List[float],
                      limit: int = 5,
                      content_id_filter: Optional[str] = None) -> List[Dict[str, Any]]:
        """
        Search for similar embeddings in Qdrant
        """
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
            with_payload=True
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

        return results

    def delete_embeddings(self, content_id: str):
        """
        Delete all embeddings for a specific content ID
        """
        # Find all points with the content_id
        search_results = self.client.scroll(
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
        point_ids = [point.id for point in search_results[0]]

        if point_ids:
            # Delete the points
            self.client.delete(
                collection_name=self.collection_name,
                points_selector=models.PointIdsList(
                    points=point_ids
                )
            )

    def get_embedding_count(self) -> int:
        """
        Get the total number of embeddings in the collection
        """
        collection_info = self.client.get_collection(self.collection_name)
        return collection_info.points_count

def cosine_similarity(vec1: List[float], vec2: List[float]) -> float:
    """
    Calculate cosine similarity between two vectors
    """
    # Convert to numpy arrays
    v1 = np.array(vec1)
    v2 = np.array(vec2)

    # Calculate cosine similarity
    dot_product = np.dot(v1, v2)
    norm_v1 = np.linalg.norm(v1)
    norm_v2 = np.linalg.norm(v2)

    if norm_v1 == 0 or norm_v2 == 0:
        return 0.0

    return float(dot_product / (norm_v1 * norm_v2))

def normalize_vector(vector: List[float]) -> List[float]:
    """
    Normalize a vector to unit length
    """
    v = np.array(vector)
    norm = np.linalg.norm(v)
    if norm == 0:
        return vector
    return (v / norm).tolist()