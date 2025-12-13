import openai
from typing import List, Optional
from ..config import settings
import logging

# Try to import numpy, fallback to pure Python implementation if it fails
try:
    import numpy as np
    NUMPY_AVAILABLE = True
except ImportError as e:
    logging.warning(f"Failed to import numpy: {e}. Using fallback implementation.")
    NUMPY_AVAILABLE = False
    # Define fallback implementations
    np = None

logger = logging.getLogger(__name__)

class EmbeddingService:
    def __init__(self):
        # Set the OpenAI API key from settings
        openai.api_key = settings.OPENAI_API_KEY

    async def generate_embedding(self, text: str, model: str = "text-embedding-ada-002") -> List[float]:
        """
        Generate embedding for the given text using OpenAI API
        """
        try:
            # Call OpenAI API to generate embedding
            response = openai.Embedding.create(
                input=text,
                model=model
            )

            # Extract the embedding from the response
            embedding = response['data'][0]['embedding']
            return embedding
        except Exception as e:
            logger.error(f"Error generating embedding: {e}")
            # Return a mock embedding in case of error
            # In production, you might want to raise the exception
            return [0.0] * 1536  # Return a zero vector with expected dimensions

    async def generate_embeddings_batch(self, texts: List[str], model: str = "text-embedding-ada-002") -> List[List[float]]:
        """
        Generate embeddings for a batch of texts
        """
        embeddings = []
        for text in texts:
            embedding = await self.generate_embedding(text, model)
            embeddings.append(embedding)
        return embeddings

    def cosine_similarity(self, vec1: List[float], vec2: List[float]) -> float:
        """
        Calculate cosine similarity between two vectors
        """
        if NUMPY_AVAILABLE and np:
            # Use numpy implementation if available
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
        else:
            # Pure Python implementation of cosine similarity
            if len(vec1) != len(vec2):
                raise ValueError("Vectors must have the same length")

            # Calculate dot product
            dot_product = sum(a * b for a, b in zip(vec1, vec2))

            # Calculate magnitudes
            magnitude1 = sum(a * a for a in vec1) ** 0.5
            magnitude2 = sum(b * b for b in vec2) ** 0.5

            if magnitude1 == 0 or magnitude2 == 0:
                return 0.0

            return dot_product / (magnitude1 * magnitude2)

    def normalize_vector(self, vector: List[float]) -> List[float]:
        """
        Normalize a vector to unit length
        """
        if NUMPY_AVAILABLE and np:
            v = np.array(vector)
            norm = np.linalg.norm(v)
            if norm == 0:
                return vector
            return (v / norm).tolist()
        else:
            # Pure Python implementation of vector normalization
            magnitude = sum(x * x for x in vector) ** 0.5
            if magnitude == 0:
                return vector
            return [x / magnitude for x in vector]

    async def get_embedding_dimensions(self, model: str = "text-embedding-ada-002") -> int:
        """
        Get the expected dimensions for embeddings from a model
        For OpenAI's text-embedding-ada-002, it's 1536
        """
        # This is a fixed value for OpenAI's ada-002 model
        if model == "text-embedding-ada-002":
            return 1536
        else:
            # For other models, we might need to check their documentation
            # For now, returning a common default
            return 1536