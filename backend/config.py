try:
    from pydantic_settings import BaseSettings
except ImportError:
    try:
        from pydantic import BaseSettings
    except ImportError:
        # For newer Pydantic versions where BaseSettings is in pydantic-settings
        raise ImportError("Please install pydantic-settings: pip install pydantic-settings")
from typing import Optional
import os
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

class Settings(BaseSettings):
    # Database settings
    DATABASE_URL: str = os.getenv("DATABASE_URL", "postgresql://user:password@localhost/dbname")

    # Qdrant settings
    QDRANT_URL: str = os.getenv("QDRANT_URL", "http://localhost:6333")
    QDRANT_API_KEY: Optional[str] = os.getenv("QDRANT_API_KEY")

    # OpenAI settings
    OPENAI_API_KEY: str = os.getenv("OPENAI_API_KEY", "")

    # Frontend settings
    REACT_APP_API_URL: str = os.getenv("REACT_APP_API_URL", "http://localhost:8000/api/v1")
    REACT_APP_WS_URL: str = os.getenv("REACT_APP_WS_URL", "ws://localhost:8000")

    # JWT settings
    SECRET_KEY: str = os.getenv("SECRET_KEY", "your-secret-key-change-in-production")
    ALGORITHM: str = "HS256"
    ACCESS_TOKEN_EXPIRE_MINUTES: int = 30

    # File upload settings
    MAX_FILE_SIZE: int = 50 * 1024 * 1024  # 50MB in bytes
    ALLOWED_EXTENSIONS: set = {"pdf", "txt", "md", "markdown"}

    class Config:
        env_file = ".env"

settings = Settings()