from fastapi import APIRouter, HTTPException, Depends
from ... import schemas
from ...database.session import get_db
from ...services.conversation_service import ConversationService
from ...services.response_service import ResponseService
from sqlalchemy.orm import Session
from uuid import UUID
import uuid

router = APIRouter()
conversation_service = ConversationService()
response_service = ResponseService()

@router.post("/start", response_model=schemas.ChatStartResponse)
async def start_chat(db: Session = Depends(get_db)):
    """
    Initialize new conversation
    Returns conversation ID
    """
    try:
        # Create a new conversation
        conversation_data = conversation_service.create_conversation(db)

        return schemas.ChatStartResponse(
            conversation_id=conversation_data["id"],
            title=conversation_data["title"]
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error starting conversation: {str(e)}")

@router.post("/{conversation_id}/message", response_model=schemas.ChatMessageResponse)
async def send_message(
    conversation_id: str,
    message_request: schemas.UserQueryCreate,
    db: Session = Depends(get_db)
):
    """
    Send user message and receive AI response
    Returns response with citations
    """
    try:
        # Add user query to the conversation
        query_data = conversation_service.add_user_query(
            conversation_id,
            message_request.query_text,
            db
        )

        # Get conversation history for context
        history = conversation_service.get_conversation_history(conversation_id, db)

        # Generate AI response using the response service
        response_data = await response_service.generate_response(
            query=message_request.query_text,
            conversation_context=history
        )

        # Add the generated response to the conversation
        response_record = conversation_service.add_generated_response(
            query_data["id"],
            response_data,
            db
        )

        # Update conversation title if this is the first query
        if len(history) <= 1:  # Only user's first message
            # Create a short title based on the query
            title = message_request.query_text[:50] + "..." if len(message_request.query_text) > 50 else message_request.query_text
            conversation_service.update_conversation_title(conversation_id, title, db)

        return schemas.ChatMessageResponse(
            response=response_data["response"],
            citations=response_data["citations"],
            conversation_id=conversation_id
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing message: {str(e)}")

@router.get("/{conversation_id}/history", response_model=schemas.ConversationHistoryResponse)
async def get_conversation_history(
    conversation_id: str,
    db: Session = Depends(get_db)
):
    """
    Retrieve conversation history
    Returns message history with timestamps
    """
    try:
        # Get conversation history
        history = conversation_service.get_conversation_history(conversation_id, db)

        # Convert to the required schema
        messages = []
        for item in history:
            messages.append(schemas.Message(
                id=item["id"],
                role=item["role"],
                content=item["content"],
                timestamp=item["timestamp"]
            ))

        return schemas.ConversationHistoryResponse(
            conversation_id=conversation_id,
            messages=messages
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving conversation history: {str(e)}")