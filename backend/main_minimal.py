from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from .config import settings
from .api.api_v1.chat_only import router as chat_router
from .database.session import engine
from . import models
import logging

logger = logging.getLogger(__name__)

# Create database tables
models.Base.metadata.create_all(bind=engine)

app = FastAPI(
    title="AI Book Chatbot API",
    description="API for the AI-powered chatbot system that answers questions about robotics books",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include only chat API routes
app.include_router(chat_router, prefix="/api/v1/chat", tags=["chat"])

@app.get("/")
def read_root():
    return {"message": "AI Book Chatbot API is running!"}

@app.get("/health")
def health_check():
    return {"status": "healthy"}

@app.post("/chat")
async def chat_endpoint(message: str):
    return {"response": f"You said: {message}"}