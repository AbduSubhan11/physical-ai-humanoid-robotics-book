# Data Model: AI Book Chatbot System

## Entity: BookContent
- **Description**: Represents chapters and sections of the book
- **Fields**:
  - id: UUID (Primary Key)
  - title: String (255 characters)
  - content: Text (Full content of the book section)
  - format: String (Enum: PDF, TEXT, MARKDOWN)
  - source_file: String (Original file name)
  - page_numbers: JSON (Page ranges for PDF content)
  - section_info: JSON (Chapter/section metadata)
  - created_at: DateTime (Timestamp of creation)
  - updated_at: DateTime (Timestamp of last update)
- **Relationships**:
  - One-to-Many with EmbeddingVector (content_id)
  - One-to-Many with Metadata (content_id)

## Entity: EmbeddingVector
- **Description**: Stores vector representations of content chunks
- **Fields**:
  - id: UUID (Primary Key)
  - content_id: UUID (Foreign Key to BookContent)
  - vector: Array[float] (Embedding values)
  - chunk_text: Text (Original text of the chunk)
  - chunk_index: Integer (Position of chunk in original content)
  - metadata: JSON (Additional metadata about the chunk)
  - created_at: DateTime (Timestamp of creation)
- **Relationships**:
  - Many-to-One with BookContent (content_id)

## Entity: UserQuery
- **Description**: Represents user questions and search parameters
- **Fields**:
  - id: UUID (Primary Key)
  - query_text: Text (The user's question)
  - user_id: UUID (Foreign Key to User, nullable for anonymous)
  - conversation_id: UUID (Foreign Key to Conversation)
  - created_at: DateTime (Timestamp of query)
- **Relationships**:
  - Many-to-One with Conversation (conversation_id)
  - One-to-Many with GeneratedResponse (query_id)

## Entity: GeneratedResponse
- **Description**: Contains AI-generated answers with citations
- **Fields**:
  - id: UUID (Primary Key)
  - query_id: UUID (Foreign Key to UserQuery)
  - response_text: Text (The AI-generated response)
  - citations: JSON (References to book sections)
  - confidence_score: Float (Confidence level of response)
  - created_at: DateTime (Timestamp of generation)
- **Relationships**:
  - Many-to-One with UserQuery (query_id)

## Entity: Conversation
- **Description**: Maintains chat history between users and system
- **Fields**:
  - id: UUID (Primary Key)
  - user_id: UUID (Foreign Key to User, nullable for anonymous)
  - title: String (Generated title for the conversation)
  - created_at: DateTime (Timestamp of creation)
  - updated_at: DateTime (Timestamp of last update)
- **Relationships**:
  - One-to-Many with UserQuery (conversation_id)

## Entity: Metadata
- **Description**: Tracks content source, timestamps, and processing status
- **Fields**:
  - id: UUID (Primary Key)
  - content_id: UUID (Foreign Key to BookContent)
  - key: String (Metadata key)
  - value: Text (Metadata value)
  - created_at: DateTime (Timestamp of creation)
- **Relationships**:
  - Many-to-One with BookContent (content_id)

## Entity: User (Optional)
- **Description**: Represents system users for account management
- **Fields**:
  - id: UUID (Primary Key)
  - email: String (User's email address)
  - username: String (Optional username)
  - is_active: Boolean (Account status)
  - created_at: DateTime (Account creation timestamp)
  - updated_at: DateTime (Last update timestamp)
- **Relationships**:
  - One-to-Many with Conversation (user_id)
  - One-to-Many with UserQuery (user_id)

## Validation Rules

### BookContent
- Title must be between 1 and 255 characters
- Content must not be empty
- Format must be one of the allowed values
- Source file must be valid

### EmbeddingVector
- Vector must have the correct dimensions
- Content_id must reference an existing BookContent
- Chunk_index must be non-negative

### UserQuery
- Query_text must be between 1 and 10000 characters
- Conversation_id must reference an existing Conversation

### GeneratedResponse
- Query_id must reference an existing UserQuery
- Confidence_score must be between 0 and 1
- Response_text must not be empty

### Conversation
- Title must be between 1 and 255 characters
- User_id may be null for anonymous conversations

## Indexes

### BookContent
- Index on (id, format)
- Index on (created_at)

### EmbeddingVector
- Index on (content_id)
- Index on (id, content_id)

### UserQuery
- Index on (conversation_id)
- Index on (created_at)

### GeneratedResponse
- Index on (query_id)
- Index on (created_at)

### Conversation
- Index on (user_id)
- Index on (updated_at)