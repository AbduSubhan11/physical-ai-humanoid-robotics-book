# Quickstart Guide: AI Book Chatbot

## Development Environment Setup

### Prerequisites
- Python 3.9 or higher
- Node.js 16 or higher
- Docker and Docker Compose
- Qdrant Cloud account (free tier)
- OpenAI API key (or alternative provider)

### Backend Setup
1. Install Python dependencies:
   ```bash
   pip install fastapi uvicorn python-multipart qdrant-client openai psycopg2-binary sqlalchemy python-jose[cryptography] passlib[bcrypt] python-dotenv celery python-multipart
   ```

2. Set up environment variables:
   ```bash
   # Create .env file with:
   DATABASE_URL=postgresql://username:password@localhost/dbname
   QDRANT_URL=https://your-cluster-url.qdrant.tech
   QDRANT_API_KEY=your-api-key
   OPENAI_API_KEY=your-openai-key
   SECRET_KEY=your-secret-key
   ALGORITHM=HS256
   ACCESS_TOKEN_EXPIRE_MINUTES=30
   ```

3. Initialize the database:
   ```bash
   # Run database migrations
   python -m alembic upgrade head
   ```

4. Start the backend:
   ```bash
   uvicorn main:app --reload --port 8000
   ```

### Frontend Setup
1. Install Node.js dependencies:
   ```bash
   cd frontend
   npm install
   ```

2. Set up environment variables:
   ```bash
   # Create .env file with:
   VITE_API_BASE_URL=http://localhost:8000
   ```

3. Start the frontend:
   ```bash
   npm run dev
   ```

### Running Background Tasks
1. Start the Celery worker for content processing:
   ```bash
   celery -A worker worker --loglevel=info
   ```

2. Start the Redis server (for Celery broker):
   ```bash
   redis-server
   ```

## API Usage Examples

### Upload Content
```bash
curl -X POST http://localhost:8000/api/v1/content/upload \
  -H "Authorization: Bearer YOUR_TOKEN" \
  -F "file=@book_chapter.pdf"
```

### Start Conversation
```bash
curl -X POST http://localhost:8000/api/v1/chat/start \
  -H "Authorization: Bearer YOUR_TOKEN"
```

### Send Message
```bash
curl -X POST http://localhost:8000/api/v1/chat/CONVERSATION_ID/message \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer YOUR_TOKEN" \
  -d '{"message": "What are the key principles of humanoid robotics?"}'
```

## Project Structure
```
project/
├── backend/
│   ├── main.py          # FastAPI application
│   ├── models/          # Database models
│   ├── schemas/         # Pydantic schemas
│   ├── database/        # Database configuration
│   ├── api/             # API routes
│   └── services/        # Business logic
├── frontend/
│   ├── src/
│   │   ├── components/  # React components
│   │   ├── pages/       # Page components
│   │   └── services/    # API service functions
├── specs/              # Specifications
├── contracts/          # API contracts
└── docs/               # Documentation
```

## Key Endpoints

### Content Management
- `POST /api/v1/content/upload` - Upload book content
- `GET /api/v1/content/processing-status/{job_id}` - Check processing status

### Chat Functionality
- `POST /api/v1/chat/start` - Start new conversation
- `POST /api/v1/chat/{conversation_id}/message` - Send message
- `GET /api/v1/chat/{conversation_id}/history` - Get conversation history

### Search
- `POST /api/v1/search` - Semantic search across book content

### Admin
- `GET /api/v1/admin/content` - List all content
- `DELETE /api/v1/admin/content/{content_id}` - Remove content