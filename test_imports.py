#!/usr/bin/env python3
"""
Test script to check if individual modules can be imported
"""

print("Testing individual imports...")

try:
    import numpy
    print(f"✓ numpy imported successfully, version: {numpy.__version__}")
except Exception as e:
    print(f"✗ Failed to import numpy: {e}")

try:
    import qdrant_client
    print("✓ qdrant_client imported successfully")
except Exception as e:
    print(f"✗ Failed to import qdrant_client: {e}")

try:
    from sklearn.feature_extraction.text import TfidfVectorizer
    print("✓ sklearn TfidfVectorizer imported successfully")
except Exception as e:
    print(f"✗ Failed to import sklearn TfidfVectorizer: {e}")

try:
    from sklearn.metrics.pairwise import cosine_similarity
    print("✓ sklearn cosine_similarity imported successfully")
except Exception as e:
    print(f"✗ Failed to import sklearn cosine_similarity: {e}")

try:
    from backend.services.vector_db import VectorDBService
    print("✓ VectorDBService imported successfully")
except Exception as e:
    print(f"✗ Failed to import VectorDBService: {e}")

try:
    from backend.services.search_service import SearchService
    print("✓ SearchService imported successfully")
except Exception as e:
    print(f"✗ Failed to import SearchService: {e}")

try:
    from backend.services.response_service import ResponseService
    print("✓ ResponseService imported successfully")
except Exception as e:
    print(f"✗ Failed to import ResponseService: {e}")

try:
    from backend.api.api_v1.chat import router
    print("✓ Chat router imported successfully")
except Exception as e:
    print(f"✗ Failed to import chat router: {e}")

print("Import test completed.")