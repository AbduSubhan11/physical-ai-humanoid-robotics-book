from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from .config import settings
from .api.api_v1 import api_router
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

# Initialize vector database service with fallback to mock
try:
    from .services.vector_db import VectorDBService
    vector_db = VectorDBService()
    vector_db.initialize_collection()
    logger.info("Successfully initialized real VectorDBService")
except Exception as e:
    logger.warning(f"Failed to initialize real VectorDBService: {e}. Using mock service.")
    from .services.vector_db_mock import VectorDBService
    vector_db = VectorDBService()
    vector_db.initialize_collection()

# Include API routes
app.include_router(api_router, prefix="/api/v1")

@app.get("/")
def read_root():
    return {"message": "AI Book Chatbot API is running!"}

@app.get("/health")
def health_check():
    # Check if we can connect to Qdrant
    qdrant_healthy = vector_db.health_check()
    return {
        "status": "healthy",
        "checks": {
            "qdrant": qdrant_healthy
        }
    }