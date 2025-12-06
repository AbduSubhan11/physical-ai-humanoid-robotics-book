from fastapi import FastAPI
from fastapi.responses import HTMLResponse
from .api import auth
from .database.database import engine, Base
from . import models # Ensure models are imported so Base knows about them

# Create database tables
Base.metadata.create_all(bind=engine)

app = FastAPI(
    title="Physical AI & Humanoid Robotics Textbook API",
    description="Backend API for the AI-Native Textbook on Physical AI & Humanoid Robotics.",
    version="0.1.0",
)

app.include_router(auth.router, prefix="/auth", tags=["auth"])

@app.get("/health", response_class=HTMLResponse, summary="Health check endpoint")
async def health():
    """Checks the health of the API."""
    return "OK"

# Placeholder for other API routes (rag, personalize, translate, generate)
