from fastapi import APIRouter, UploadFile, File, HTTPException, BackgroundTasks, Depends
from typing import Optional
from ... import schemas
from ...database.session import get_db
from ...services.content_service import ContentService
from sqlalchemy.orm import Session

router = APIRouter()
content_service = ContentService()

@router.post("/upload", response_model=schemas.ContentUploadResponse)
async def upload_content(
    file: UploadFile = File(...),
    db: Session = Depends(get_db)
):
    """
    Upload book chapters (text, PDF, Markdown)
    Returns upload status and processing job ID
    """
    try:
        # Process the upload using the content service
        result = await content_service.process_upload(file, db)

        return schemas.ContentUploadResponse(
            job_id=result["content_id"],
            status=result["status"]
        )
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

@router.get("/processing-status/{job_id}", response_model=schemas.ProcessingStatusResponse)
async def get_processing_status(job_id: str):
    """
    Check status of content processing job
    Returns progress percentage and status
    """
    # In a real implementation, we would track processing jobs
    # For now, we'll assume the job is completed since we process synchronously
    return schemas.ProcessingStatusResponse(
        job_id=job_id,
        status="completed",
        progress=1.0,
        message="Processing completed successfully"
    )