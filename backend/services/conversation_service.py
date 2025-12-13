from typing import List, Dict, Any, Optional
from sqlalchemy.orm import Session
from uuid import UUID
import uuid
from datetime import datetime
from ..models import Conversation as ConversationModel, UserQuery, GeneratedResponse
from ..schemas import UserQueryCreate, GeneratedResponseCreate

class ConversationService:
    def __init__(self):
        pass

    def create_conversation(self, db: Session, user_id: Optional[UUID] = None) -> Dict[str, Any]:
        """
        Create a new conversation
        """
        conversation = ConversationModel(
            user_id=user_id,
            title="New Conversation"
        )

        db.add(conversation)
        db.commit()
        db.refresh(conversation)

        return {
            "id": str(conversation.id),
            "title": conversation.title,
            "created_at": conversation.created_at.isoformat()
        }

    def get_conversation_history(self, conversation_id: str, db: Session) -> List[Dict[str, Any]]:
        """
        Get conversation history
        """
        # Get all queries in the conversation
        queries = db.query(UserQuery).filter(
            UserQuery.conversation_id == conversation_id
        ).order_by(UserQuery.created_at).all()

        history = []

        for query in queries:
            # Add user query to history
            history.append({
                "id": str(query.id),
                "role": "user",
                "content": query.query_text,
                "timestamp": query.created_at.isoformat()
            })

            # Get the corresponding response
            response = db.query(GeneratedResponse).filter(
                GeneratedResponse.query_id == query.id
            ).first()

            if response:
                history.append({
                    "id": str(response.id),
                    "role": "assistant",
                    "content": response.response_text,
                    "timestamp": response.created_at.isoformat()
                })

        return history

    def add_user_query(self, conversation_id: str, query_text: str, db: Session) -> Dict[str, Any]:
        """
        Add a user query to the conversation
        """
        user_query = UserQuery(
            query_text=query_text,
            conversation_id=conversation_id
        )

        db.add(user_query)
        db.commit()
        db.refresh(user_query)

        return {
            "id": str(user_query.id),
            "query_text": user_query.query_text,
            "conversation_id": str(user_query.conversation_id),
            "created_at": user_query.created_at.isoformat()
        }

    def add_generated_response(self, query_id: str, response_data: Dict[str, Any], db: Session) -> Dict[str, Any]:
        """
        Add a generated response to the conversation
        """
        generated_response = GeneratedResponse(
            query_id=query_id,
            response_text=response_data["response"],
            citations=response_data["citations"],
            confidence_score=response_data["confidence_score"]
        )

        db.add(generated_response)
        db.commit()
        db.refresh(generated_response)

        return {
            "id": str(generated_response.id),
            "response_text": generated_response.response_text,
            "citations": generated_response.citations,
            "confidence_score": generated_response.confidence_score,
            "created_at": generated_response.created_at.isoformat()
        }

    def update_conversation_title(self, conversation_id: str, title: str, db: Session):
        """
        Update conversation title based on the first query
        """
        conversation = db.query(ConversationModel).filter(
            ConversationModel.id == conversation_id
        ).first()

        if conversation:
            conversation.title = title
            conversation.updated_at = datetime.utcnow()
            db.commit()