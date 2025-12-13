from openai import OpenAI
from typing import List, Dict, Any
from ..config import settings
import logging
from ..utils.text_processing import extract_sections

# Try to import SearchService, but handle the import error gracefully
try:
    from ..services.search_service import SearchService
    SEARCH_SERVICE_AVAILABLE = True
except ImportError as e:
    logging.warning(f"Failed to import SearchService: {e}. Chat functionality may be limited.")
    SEARCH_SERVICE_AVAILABLE = False
    SearchService = None

logger = logging.getLogger(__name__)

class ResponseService:
    def __init__(self):
        if SEARCH_SERVICE_AVAILABLE and SearchService:
            self.search_service = SearchService()
        else:
            self.search_service = None
            logger.warning("SearchService not available, response generation will be limited.")

    async def generate_response(
        self,
        query: str,
        conversation_context: List[Dict[str, str]] = None
    ) -> Dict[str, Any]:
        """
        Generate a response based on the query and relevant context from book content
        """
        try:
            if not self.search_service:
                # If search service is not available, return a default response
                return {
                    "response": "Search service is currently unavailable. Please ensure all dependencies are properly installed.",
                    "citations": [],
                    "confidence_score": 0.0
                }

            # Find relevant context from the book content
            search_results = await self.search_service.find_relevant_context(query, num_results=5)

            if not search_results:
                return {
                    "response": "I couldn't find relevant information in the book content to answer your question.",
                    "citations": [],
                    "confidence_score": 0.0
                }

            # Prepare context for the LLM
            context_parts = []
            citations = []

            for result in search_results:
                content = result["content"]
                source = result["source"]

                if content.strip():  # Only add non-empty content
                    context_parts.append(f"Relevant information: {content}")

                    # Create citation
                    citation = {
                        "section": source.get("title", "Unknown Section"),
                        "page": source.get("page", "Unknown"),
                        "confidence": self.search_service.calculate_confidence_score(result["score"])
                    }
                    citations.append(citation)

            # Combine all context
            context = "\n\n".join(context_parts)

            # Prepare the prompt for the LLM
            # Include conversation context if available
            conversation_history = ""
            if conversation_context:
                conversation_history = "Previous conversation:\n"
                for msg in conversation_context[-5:]:  # Use last 5 messages as context
                    role = msg.get("role", "user")
                    content = msg.get("content", "")
                    conversation_history += f"{role.capitalize()}: {content}\n"

            prompt = f"""
            You are an AI assistant for a robotics book. Answer the user's question based ONLY on the provided book content.
            Do not use any external knowledge or information not present in the provided context.
            If the information is not available in the provided context, clearly state that.

            {conversation_history}

            Context from the book:
            {context}

            User's question: {query}

            Instructions:
            1. Answer based only on the provided context
            2. If the answer is not in the context, say so clearly
            3. Provide specific citations to book sections when possible
            4. Keep the response concise and helpful
            5. If multiple relevant sections exist, synthesize the information
            """

            # Call OpenAI API to generate response
            client = OpenAI(api_key=settings.OPENAI_API_KEY)
            response = client.chat.completions.create(
                model="gpt-3.5-turbo",  # You might want to use gpt-4 for better results
                messages=[
                    {"role": "system", "content": "You are a helpful assistant that answers questions based only on provided context from a robotics book. Do not use any external knowledge."},
                    {"role": "user", "content": prompt}
                ],
                max_tokens=500,
                temperature=0.3,  # Lower temperature for more consistent, factual responses
                top_p=0.9,
                frequency_penalty=0.1,
                presence_penalty=0.1
            )

            # Extract the response
            ai_response = response.choices[0].message.content.strip()

            # Calculate overall confidence as average of citation confidences
            if citations:
                avg_confidence = sum([c['confidence'] for c in citations]) / len(citations)
            else:
                avg_confidence = 0.0

            return {
                "response": ai_response,
                "citations": citations,
                "confidence_score": avg_confidence
            }

        except Exception as e:
            logger.error(f"Error generating response: {e}")
            return {
                "response": "I encountered an error processing your request. Please try again.",
                "citations": [],
                "confidence_score": 0.0
            }

    async def validate_response_relevance(
        self,
        query: str,
        response: str,
        context: List[Dict[str, Any]]
    ) -> bool:
        """
        Validate if the response is relevant to the query and context
        """
        try:
            # Simple validation: check if response contains information related to query
            query_lower = query.lower()
            response_lower = response.lower()

            # Check if response addresses the main topic of the query
            query_words = query_lower.split()
            response_relevant = any(word in response_lower for word in query_words[:3])  # Check first few words

            return response_relevant

        except Exception as e:
            logger.error(f"Error validating response relevance: {e}")
            return False