from sqlalchemy import Column, Integer, String, Text, DateTime, Boolean, ForeignKey, JSON
from sqlalchemy.types import TypeDecorator, CHAR
import uuid
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import relationship
from datetime import datetime

# Custom UUID type that works with both PostgreSQL and SQLite
class GUID(TypeDecorator):
    """Platform-independent GUID type.
    Uses PostgreSQL's UUID type, otherwise uses CHAR(32), storing as stringified hex values.
    """
    impl = CHAR
    cache_ok = True

    def load_dialect_impl(self, dialect):
        if dialect.name == 'postgresql':
            return dialect.type_descriptor(UUID())
        else:
            return dialect.type_descriptor(CHAR(32))

    def process_bind_param(self, value, dialect):
        if value is None:
            return value
        elif dialect.name == 'postgresql':
            return value
        else:
            if not isinstance(value, uuid.UUID):
                return "%.32x" % uuid.UUID(value).int
            else:
                # hexstring
                return "%.32x" % value.int

    def process_result_value(self, value, dialect):
        if value is None:
            return value
        else:
            if not isinstance(value, uuid.UUID):
                value = uuid.UUID(value)
            return value

# Base class for all models
Base = declarative_base()

def generate_uuid():
    return str(uuid.uuid4())

class User(Base):
    __tablename__ = "users"

    id = Column(GUID, primary_key=True, default=generate_uuid)
    email = Column(String, unique=True, index=True)
    username = Column(String, unique=True, index=True)
    hashed_password = Column(String)
    is_active = Column(Boolean, default=True)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)

class BookContent(Base):
    __tablename__ = "book_content"

    id = Column(GUID, primary_key=True, default=generate_uuid)
    title = Column(String(255), nullable=False)
    content = Column(Text, nullable=False)
    format = Column(String, nullable=False)  # 'PDF', 'TEXT', 'MARKDOWN'
    source_file = Column(String)
    page_numbers = Column(JSON)  # For PDF content
    section_info = Column(JSON)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)

    # Relationship with EmbeddingVector
    embeddings = relationship("EmbeddingVector", back_populates="book_content")
    # Relationship with Metadata
    metadata_assoc = relationship("Metadata", back_populates="book_content")

class EmbeddingVector(Base):
    __tablename__ = "embedding_vectors"

    id = Column(GUID, primary_key=True, default=generate_uuid)
    content_id = Column(GUID, ForeignKey("book_content.id"), nullable=False)
    vector = Column(JSON, nullable=False)  # Store as JSON for flexibility
    chunk_text = Column(Text, nullable=False)
    chunk_index = Column(Integer, nullable=False)
    metadata_json = Column(JSON)
    created_at = Column(DateTime, default=datetime.utcnow)

    # Relationship with BookContent
    book_content = relationship("BookContent", back_populates="embeddings")

class Conversation(Base):
    __tablename__ = "conversations"

    id = Column(GUID, primary_key=True, default=generate_uuid)
    user_id = Column(GUID, ForeignKey("users.id"), nullable=True)  # Nullable for anonymous users
    title = Column(String(255), nullable=False)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)

    # Relationship with User
    user = relationship("User")
    # Relationship with UserQuery
    queries = relationship("UserQuery", back_populates="conversation")

class UserQuery(Base):
    __tablename__ = "user_queries"

    id = Column(GUID, primary_key=True, default=generate_uuid)
    query_text = Column(Text, nullable=False)
    user_id = Column(GUID, ForeignKey("users.id"), nullable=True)  # Nullable for anonymous users
    conversation_id = Column(GUID, ForeignKey("conversations.id"), nullable=False)
    created_at = Column(DateTime, default=datetime.utcnow)

    # Relationships
    user = relationship("User")
    conversation = relationship("Conversation", back_populates="queries")
    # Relationship with GeneratedResponse
    responses = relationship("GeneratedResponse", back_populates="query")

class GeneratedResponse(Base):
    __tablename__ = "generated_responses"

    id = Column(GUID, primary_key=True, default=generate_uuid)
    query_id = Column(GUID, ForeignKey("user_queries.id"), nullable=False)
    response_text = Column(Text, nullable=False)
    citations = Column(JSON)  # References to book sections
    confidence_score = Column(Integer)  # Confidence level of response (0-100)
    created_at = Column(DateTime, default=datetime.utcnow)

    # Relationship with UserQuery
    query = relationship("UserQuery", back_populates="responses")

class Metadata(Base):
    __tablename__ = "metadata"

    id = Column(GUID, primary_key=True, default=generate_uuid)
    content_id = Column(GUID, ForeignKey("book_content.id"), nullable=False)
    key = Column(String, nullable=False)
    value = Column(Text, nullable=False)
    created_at = Column(DateTime, default=datetime.utcnow)

    # Relationship with BookContent
    book_content = relationship("BookContent", back_populates="metadata_assoc")