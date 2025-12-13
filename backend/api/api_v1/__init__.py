from fastapi import APIRouter
import logging

logger = logging.getLogger(__name__)

api_router = APIRouter()

# Import and include only essential routes for chat functionality with error handling
try:
    import importlib
    chat = importlib.import_module('.chat', package=__name__)
    api_router.include_router(chat.router, prefix="/chat", tags=["chat"])
    logger.info("Successfully loaded chat routes")
except ImportError as e:
    logger.warning(f"Could not import chat routes: {e}")
except Exception as e:
    logger.warning(f"Error loading chat routes: {e}")

try:
    import importlib
    search = importlib.import_module('.search', package=__name__)
    api_router.include_router(search.router, prefix="/search", tags=["search"])
    logger.info("Successfully loaded search routes")
except ImportError as e:
    logger.warning(f"Could not import search routes: {e}")
except Exception as e:
    logger.warning(f"Error loading search routes: {e}")

# Only import other routes if needed
try:
    from . import auth
    api_router.include_router(auth.router, prefix="/auth", tags=["auth"])
    logger.info("Successfully loaded auth routes")
except ImportError as e:
    logger.warning(f"Could not import auth routes: {e}")

try:
    from . import content
    api_router.include_router(content.router, prefix="/content", tags=["content"])
    logger.info("Successfully loaded content routes")
except ImportError as e:
    logger.warning(f"Could not import content routes: {e}")

try:
    from . import admin
    api_router.include_router(admin.router, prefix="/admin", tags=["admin"])
    logger.info("Successfully loaded admin routes")
except ImportError as e:
    logger.warning(f"Could not import admin routes: {e}")