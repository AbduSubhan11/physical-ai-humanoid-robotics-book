from backend.main import app
import uvicorn

if __name__ == "__main__":
    uvicorn.run(
        "backend.main:app",
        host="0.0.0.0",  # Make it accessible from other machines too
        port=8000,
        reload=True  # Enable auto-reload during development
    )