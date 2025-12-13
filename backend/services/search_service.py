from typing import List, Dict, Any
from ..services.embedding_service import EmbeddingService
from ..config import settings
import logging

# Try to import real VectorDBService, fallback to mock if numpy fails
try:
    from ..services.vector_db import VectorDBService
    import numpy as np
    from sklearn.feature_extraction.text import TfidfVectorizer
    from sklearn.metrics.pairwise import cosine_similarity
except ImportError as e:
    # Import the mock service if numpy fails
    from ..services.vector_db_mock import VectorDBService
    import numpy as np  # This will likely fail too, so we'll handle it
    # Define placeholders for sklearn components if they fail to import
    TfidfVectorizer = None
    cosine_similarity = None
    logging.warning(f"Failed to import numpy/scikit-learn components: {e}. Using limited functionality.")

logger = logging.getLogger(__name__)

class SearchService:
    def __init__(self):
        self.embedding_service = EmbeddingService()
        self.vector_db_service = VectorDBService()

    async def semantic_search(
        self,
        query: str,
        top_k: int = 5,
        content_id_filter: str = None,
        advanced_ranking: bool = True
    ) -> List[Dict[str, Any]]:
        """
        Perform semantic search against stored content with advanced ranking
        """
        try:
            # Generate embedding for the query
            query_embedding = await self.embedding_service.generate_embedding(query)

            # Search in vector database
            search_results = self.vector_db_service.search_similar(
                query_embedding=query_embedding,
                limit=top_k * 2,  # Get more results for advanced ranking
                content_id_filter=content_id_filter
            )

            if advanced_ranking and len(search_results) > 1:
                # Apply advanced ranking algorithm
                ranked_results = self._advanced_ranking(query, search_results)
            else:
                ranked_results = search_results

            # Take top_k results after ranking
            ranked_results = ranked_results[:top_k]

            # Format results with detailed citations and metadata
            formatted_results = []
            for result in ranked_results:
                # Calculate confidence score
                confidence_score = self.calculate_confidence_score(result["score"])

                formatted_result = {
                    "content": result["content"],
                    "source": {
                        "content_id": result["content_id"],
                        "chunk_index": result["chunk_index"],
                        "title": result["metadata"].get("title", "Unknown"),
                        "section": result["metadata"].get("section", "Unknown"),
                        "page": result["metadata"].get("page", "Unknown"),
                        "source_file": result["metadata"].get("source_file", "Unknown"),
                        **result["metadata"]  # Include other metadata
                    },
                    "score": result["score"],
                    "confidence_score": confidence_score
                }
                formatted_results.append(formatted_result)

            logger.info(f"Semantic search returned {len(formatted_results)} results")
            return formatted_results

        except Exception as e:
            logger.error(f"Error performing semantic search: {e}")
            # Return empty list in case of error
            return []

    def _advanced_ranking(self, query: str, results: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Apply advanced ranking algorithm combining semantic similarity with keyword matching
        """
        if not results:
            return results

        # If sklearn components are not available, return results sorted by semantic score
        if TfidfVectorizer is None or cosine_similarity is None:
            results.sort(key=lambda x: x["score"], reverse=True)
            return results

        # Extract content texts for TF-IDF analysis
        contents = [result["content"] for result in results]

        try:
            # Use TF-IDF to get keyword relevance
            vectorizer = TfidfVectorizer(
                stop_words='english',
                ngram_range=(1, 2),  # Include unigrams and bigrams
                max_features=1000
            )

            # Fit on contents and transform query
            content_vectors = vectorizer.fit_transform(contents)
            query_vector = vectorizer.transform([query])

            # Calculate TF-IDF based similarity
            tfidf_similarities = cosine_similarity(query_vector, content_vectors).flatten()

            # Combine semantic similarity (from vector DB) with TF-IDF similarity
            for i, result in enumerate(results):
                semantic_score = result["score"]
                tfidf_score = tfidf_similarities[i]

                # Weighted combination (can be tuned)
                combined_score = 0.7 * semantic_score + 0.3 * tfidf_score
                result["combined_score"] = combined_score

            # Sort by combined score
            results.sort(key=lambda x: x["combined_score"], reverse=True)

            return results
        except Exception as e:
            logging.warning(f"Advanced ranking failed: {e}. Falling back to semantic score sorting.")
            # If TF-IDF fails, just return original results sorted by semantic score
            results.sort(key=lambda x: x["score"], reverse=True)
            return results

    async def find_relevant_context(
        self,
        query: str,
        num_results: int = 3
    ) -> List[Dict[str, Any]]:
        """
        Find relevant context for a query to use in response generation
        """
        results = await self.semantic_search(query, top_k=num_results)
        return results

    def calculate_confidence_score(self, similarity_score: float) -> float:
        """
        Convert similarity score to confidence score (0-1 scale)
        """
        # The similarity score from Qdrant is already in the range [0, 1] for cosine similarity
        # with some adjustment to ensure it's properly normalized
        return max(0.0, min(1.0, similarity_score))

    async def get_related_content(
        self,
        content_id: str,
        top_k: int = 5
    ) -> List[Dict[str, Any]]:
        """
        Get content related to a specific piece of content
        """
        try:
            # Get the original content
            original_content = self.vector_db_service.search_similar(
                query_embedding=[0.1] * 1536,  # Placeholder - in practice, we'd get the actual embedding
                limit=1,
                content_id_filter=content_id
            )

            if not original_content:
                return []

            # Use the content as a query to find similar content
            original_text = original_content[0]["content"]
            related_results = await self.semantic_search(
                query=original_text,
                top_k=top_k + 1,  # Get one more to exclude the original
                content_id_filter=None  # Don't filter by the same content ID
            )

            # Filter out the original content
            related_results = [
                result for result in related_results
                if result["source"]["content_id"] != content_id
            ][:top_k]

            return related_results
        except Exception as e:
            logger.error(f"Error getting related content: {e}")
            return []