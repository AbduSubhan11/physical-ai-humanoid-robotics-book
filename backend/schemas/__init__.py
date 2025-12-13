from pydantic import BaseModel, Field
from typing import Optional, List, Dict, Any
from datetime import datetime
from uuid import UUID

# User schemas
class UserBase(BaseModel):
    email: str
    username: Optional[str] = None
    is_active: Optional[bool] = True

class UserCreate(UserBase):
    password: str

class UserUpdate(UserBase):
    password: Optional[str] = None

class User(UserBase):
    id: UUID
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True

# BookContent schemas
class BookContentBase(BaseModel):
    title: str
    content: str
    format: str  # 'PDF', 'TEXT', 'MARKDOWN'
    source_file: Optional[str] = None
    page_numbers: Optional[Dict[str, Any]] = None
    section_info: Optional[Dict[str, Any]] = None

class BookContentCreate(BookContentBase):
    pass

class BookContentUpdate(BaseModel):
    title: Optional[str] = None
    content: Optional[str] = None
    section_info: Optional[Dict[str, Any]] = None

class BookContent(BookContentBase):
    id: UUID
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True

# EmbeddingVector schemas
class EmbeddingVectorBase(BaseModel):
    content_id: UUID
    vector: List[float]
    chunk_text: str
    chunk_index: int
    metadata: Optional[Dict[str, Any]] = None

class EmbeddingVectorCreate(EmbeddingVectorBase):
    pass

class EmbeddingVector(EmbeddingVectorBase):
    id: UUID
    created_at: datetime

    class Config:
        from_attributes = True

# Conversation schemas
class ConversationBase(BaseModel):
    title: str

class ConversationCreate(ConversationBase):
    pass

class ConversationUpdate(BaseModel):
    title: Optional[str] = None

class Conversation(ConversationBase):
    id: UUID
    user_id: Optional[UUID] = None
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True

# UserQuery schemas
class UserQueryBase(BaseModel):
    query_text: str
    conversation_id: UUID

class UserQueryCreate(UserQueryBase):
    pass

class UserQuery(UserQueryBase):
    id: UUID
    user_id: Optional[UUID] = None
    created_at: datetime

    class Config:
        from_attributes = True

# GeneratedResponse schemas
class GeneratedResponseBase(BaseModel):
    query_id: UUID
    response_text: str
    citations: Optional[List[Dict[str, Any]]] = None
    confidence_score: Optional[int] = None

class GeneratedResponseCreate(GeneratedResponseBase):
    pass

class GeneratedResponse(GeneratedResponseBase):
    id: UUID
    created_at: datetime

    class Config:
        from_attributes = True

# Metadata schemas
class MetadataBase(BaseModel):
    content_id: UUID
    key: str
    value: str

class MetadataCreate(MetadataBase):
    pass

class Metadata(MetadataBase):
    id: UUID
    created_at: datetime

    class Config:
        from_attributes = True

# Content ingestion response
class ContentUploadResponse(BaseModel):
    job_id: str
    status: str

# Content processing status response
class ProcessingStatusResponse(BaseModel):
    job_id: str
    status: str
    progress: float
    message: Optional[str] = None

# Chat start response
class ChatStartResponse(BaseModel):
    conversation_id: str
    title: str

# Chat message response
class ChatMessageResponse(BaseModel):
    response: str
    citations: List[Dict[str, Any]]
    conversation_id: str

# Search result schemas
class SearchResult(BaseModel):
    content: str
    source: Dict[str, Any]
    score: float

class SearchRequest(BaseModel):
    query: str
    top_k: Optional[int] = 5

class SearchResponse(BaseModel):
    results: List[SearchResult]
    query: str

# Conversation history response
class Message(BaseModel):
    id: UUID
    role: str  # 'user' or 'assistant'
    content: str
    timestamp: datetime

class ConversationHistoryResponse(BaseModel):
    conversation_id: str
    messages: List[Message]