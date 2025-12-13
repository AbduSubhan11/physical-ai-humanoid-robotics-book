from fastapi import APIRouter, Depends
from ... import schemas
from ...services.search_service import SearchService
from ...database.session import get_db
from sqlalchemy.orm import Session

router = APIRouter()
search_service = SearchService()

@router.post("/", response_model=schemas.SearchResponse)
async def semantic_search(
    search_request: schemas.SearchRequest,
    db: Session = Depends(get_db)
):
    """
    Perform semantic search against book content
    Returns relevant content chunks with metadata
    """
    try:
        # Perform semantic search using the search service
        results = await search_service.semantic_search(
            query=search_request.query,
            top_k=search_request.top_k,
            advanced_ranking=True
        )

        # Format results to match the schema
        formatted_results = []
        for result in results:
            formatted_results.append(
                schemas.SearchResult(
                    content=result["content"],
                    source=result["source"],
                    score=result["score"]
                )
            )

        return schemas.SearchResponse(
            results=formatted_results,
            query=search_request.query
        )
    except Exception as e:
        from fastapi import HTTPException
        raise HTTPException(status_code=500, detail=f"Error performing search: {str(e)}")

@router.post("/related/{content_id}")
async def get_related_content(
    content_id: str,
    top_k: int = 5,
    db: Session = Depends(get_db)
):
    """
    Get content related to a specific piece of content
    """
    try:
        results = await search_service.get_related_content(
            content_id=content_id,
            top_k=top_k
        )

        # Format results to match the schema
        formatted_results = []
        for result in results:
            formatted_results.append(
                schemas.SearchResult(
                    content=result["content"],
                    source=result["source"],
                    score=result["score"]
                )
            )

        return schemas.SearchResponse(
            results=formatted_results,
            query=f"Related to content {content_id}"
        )
    except Exception as e:
        from fastapi import HTTPException
        raise HTTPException(status_code=500, detail=f"Error getting related content: {str(e)}")