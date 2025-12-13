"""
Application startup script to initialize services and perform any required setup
"""
from .services.vector_db import VectorDBService
from .config import settings

def initialize_services():
    """
    Initialize all required services at startup
    """
    print("Initializing services...")

    # Initialize vector database service
    vector_db = VectorDBService()
    vector_db.initialize_collection()
    print("✓ Vector database initialized")

    print("All services initialized successfully!")

if __name__ == "__main__":
    initialize_services()