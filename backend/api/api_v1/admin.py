from fastapi import APIRouter, HTTPException, Depends
from ... import schemas
from ...database.session import get_db
from ...services.content_service import ContentService
from sqlalchemy.orm import Session

router = APIRouter()
content_service = ContentService()

@router.get("/content", response_model=dict)
async def list_content(db: Session = Depends(get_db)):
    """
    List all book content in the system
    Requires admin authentication
    """
    try:
        content_list = content_service.get_content_list(db)
        return {
            "content_list": content_list
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error listing content: {str(e)}")

@router.delete("/content/{content_id}")
async def remove_content(content_id: str, db: Session = Depends(get_db)):
    """
    Remove specific book content from the system
    Requires admin authentication
    """
    try:
        success = content_service.delete_content(content_id, db)
        if not success:
            raise HTTPException(status_code=404, detail="Content not found")

        return {"message": "Content removed successfully"}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error removing content: {str(e)}")