from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from ..config import settings
import os

# Use SQLite for development if PostgreSQL is not available
DATABASE_URL = settings.DATABASE_URL

# Check if we're using a PostgreSQL URL, and if so, try to use SQLite instead for development
if DATABASE_URL.startswith("postgresql://"):
    # Use SQLite for development instead of PostgreSQL
    DATABASE_URL = "sqlite:///./ai_book_chatbot.db"

# Create the database engine
engine = create_engine(
    DATABASE_URL,
    pool_pre_ping=True,
    pool_recycle=300,
    connect_args={"check_same_thread": False}  # Required for SQLite
)

# Create a configured "SessionLocal" class
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

# Create a Base class for declarative models
Base = declarative_base()

def get_db():
    """Dependency to get database session"""
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()