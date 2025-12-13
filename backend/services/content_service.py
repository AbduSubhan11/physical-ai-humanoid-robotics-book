import asyncio
from typing import Optional, Dict, Any, List
from pathlib import Path
import pdfplumber
import markdown
from fastapi import UploadFile, HTTPException
from ..models import BookContent, EmbeddingVector
from ..database.session import get_db
from ..utils.text_processing import extract_text_from_markdown, clean_text
from ..utils.chunking import chunk_content
from ..utils.embedding import EmbeddingManager
from ..services.embedding_service import EmbeddingService
from ..config import settings
import tempfile
import os
from sqlalchemy.orm import Session
import PyPDF2

class ContentService:
    def __init__(self):
        self.embedding_manager = EmbeddingManager()
        self.embedding_service = EmbeddingService()
        self.embedding_manager.create_collection()  # Initialize the collection

    async def process_upload(self, file: UploadFile, db: Session) -> Dict[str, Any]:
        """
        Process an uploaded file and store its content
        """
        # Validate file type
        file_extension = file.filename.split(".")[-1].lower()
        if file_extension not in settings.ALLOWED_EXTENSIONS:
            raise HTTPException(
                status_code=400,
                detail=f"Invalid file format. Only {', '.join(settings.ALLOWED_EXTENSIONS)} files are allowed."
            )

        # Validate file size
        file_content = await file.read()
        if len(file_content) > settings.MAX_FILE_SIZE:
            raise HTTPException(
                status_code=400,
                detail=f"File too large. Maximum size is {settings.MAX_FILE_SIZE} bytes."
            )

        # Reset file pointer after reading
        await file.seek(0)

        # Extract text based on file type
        if file_extension == "pdf":
            text_content = await self._extract_text_from_pdf(file)
        elif file_extension in ["md", "markdown"]:
            text_content = await self._extract_text_from_markdown(file)
        elif file_extension == "txt":
            text_content = await self._extract_text_from_txt(file)
        else:
            raise HTTPException(
                status_code=400,
                detail=f"Unsupported file type: {file_extension}"
            )

        # Clean the text
        cleaned_text = clean_text(text_content)

        # Create BookContent record
        book_content = BookContent(
            title=file.filename,
            content=cleaned_text,
            format=file_extension.upper(),
            source_file=file.filename
        )

        db.add(book_content)
        db.commit()
        db.refresh(book_content)

        # Chunk the content and generate embeddings
        chunked_content = chunk_content(cleaned_text)
        for chunk_data in chunked_content:
            chunk_text = chunk_data["text"]
            chunk_index = chunk_data["chunk_index"]

            # Generate embedding for the chunk using the embedding service
            embedding = await self.embedding_service.generate_embedding(chunk_text)

            # Store embedding in Qdrant
            embedding_id = self.embedding_manager.store_embedding(
                content_id=str(book_content.id),
                chunk_text=chunk_text,
                embedding=embedding,
                chunk_index=chunk_index,
                metadata={
                    "source_file": file.filename,
                    "chunk_size": len(chunk_text),
                    "start_pos": chunk_data["start_pos"],
                    "end_pos": chunk_data["end_pos"]
                }
            )

            # Store embedding reference in database (for tracking purposes)
            embedding_vector = EmbeddingVector(
                content_id=book_content.id,
                vector=embedding,
                chunk_text=chunk_text,
                chunk_index=chunk_index
            )
            db.add(embedding_vector)

        db.commit()

        return {
            "content_id": str(book_content.id),
            "title": book_content.title,
            "chunks_processed": len(chunks),
            "status": "completed"
        }

    async def _extract_text_from_pdf(self, file: UploadFile) -> str:
        """
        Extract text from PDF file
        """
        # Write uploaded file to temporary location for processing
        content = await file.read()
        # Reset file pointer after reading
        await file.seek(0)

        # Create a BytesIO object from the content for PyPDF2
        import io
        pdf_file = io.BytesIO(content)

        try:
            # Extract text using PyPDF2
            pdf_reader = PyPDF2.PdfReader(pdf_file)
            text = ""
            for page_num in range(len(pdf_reader.pages)):
                page = pdf_reader.pages[page_num]
                text += page.extract_text() + "\n"

            return text
        except Exception as e:
            # If PyPDF2 fails, try with pdfplumber as fallback
            with tempfile.NamedTemporaryFile(delete=False, suffix=".pdf") as temp_file:
                temp_file.write(content)
                temp_file_path = temp_file.name

            try:
                text = ""
                with pdfplumber.open(temp_file_path) as pdf:
                    for page in pdf.pages:
                        page_text = page.extract_text()
                        if page_text:
                            text += page_text + "\n"
                return text
            finally:
                # Clean up temporary file
                os.unlink(temp_file_path)

    async def _extract_text_from_markdown(self, file: UploadFile) -> str:
        """
        Extract text from Markdown file
        """
        content = await file.read()
        text = content.decode('utf-8')
        return extract_text_from_markdown(text)

    async def _extract_text_from_txt(self, file: UploadFile) -> str:
        """
        Extract text from TXT file
        """
        content = await file.read()
        return content.decode('utf-8')

    def get_content_list(self, db: Session) -> List[Dict[str, Any]]:
        """
        Get list of all book content in the system
        """
        contents = db.query(BookContent).all()
        return [
            {
                "id": str(content.id),
                "title": content.title,
                "format": content.format,
                "created_at": content.created_at.isoformat()
            }
            for content in contents
        ]

    def delete_content(self, content_id: str, db: Session) -> bool:
        """
        Delete content and its embeddings
        """
        # Remove from database
        content = db.query(BookContent).filter(BookContent.id == content_id).first()
        if not content:
            return False

        db.delete(content)
        db.commit()

        # Remove from vector database
        self.embedding_manager.delete_embeddings(content_id)

        return True