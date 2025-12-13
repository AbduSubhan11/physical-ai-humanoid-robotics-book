import re
from typing import List, Dict, Any

def chunk_text(text: str, chunk_size: int = 1000, overlap: int = 100) -> List[str]:
    """
    Split text into overlapping chunks of specified size
    """
    if len(text) <= chunk_size:
        return [text]

    chunks = []
    start = 0

    while start < len(text):
        end = start + chunk_size

        # If this is not the last chunk, try to break at sentence boundary
        if end < len(text):
            # Look for sentence endings near the end
            sentence_end = max(text.rfind('.', start, end),
                             text.rfind('!', start, end),
                             text.rfind('?', start, end))

            if sentence_end > start + chunk_size // 2:  # Only if sentence end is reasonably close
                end = sentence_end + 1
            else:
                # If no sentence end found, break at word boundary
                word_end = text.rfind(' ', start + chunk_size // 2, end)
                if word_end > start + chunk_size // 2:
                    end = word_end

        chunks.append(text[start:end].strip())

        # Move start position with overlap
        start = end - overlap if end < len(text) else len(text)

        # Adjust start to avoid empty chunks
        if start <= start + chunk_size // 2 and start < len(text):
            # If overlap would create a very small chunk, just move to next position
            start = end

    # Filter out empty chunks
    return [chunk for chunk in chunks if chunk]

def extract_text_from_markdown(markdown_content: str) -> str:
    """
    Extract plain text from markdown content
    """
    # Remove markdown formatting but keep the content
    # Remove headers
    text = re.sub(r'^#+\s+', '', markdown_content, flags=re.MULTILINE)
    # Remove bold/italic
    text = re.sub(r'\*{1,2}([^*]+)\*{1,2}', r'\1', text)
    text = re.sub(r'_{1,2}([^_]+)_{1,2}', r'\1', text)
    # Remove code blocks
    text = re.sub(r'`{3}[\s\S]*?`{3}', '', text)
    text = re.sub(r'`[^`]*`', '', text)
    # Remove links but keep link text
    text = re.sub(r'\[([^\]]+)\]\([^)]+\)', r'\1', text)
    # Remove images
    text = re.sub(r'!\[[^\]]*\]\([^)]+\)', '', text)
    # Remove blockquotes
    text = re.sub(r'^>\s+', '', text, flags=re.MULTILINE)
    # Remove lists
    text = re.sub(r'^\s*[\*\-\+]\s+', '', text, flags=re.MULTILINE)
    text = re.sub(r'^\s*\d+\.\s+', '', text, flags=re.MULTILINE)

    # Clean up extra whitespace
    text = re.sub(r'\n\s*\n', '\n\n', text)
    text = text.strip()

    return text

def clean_text(text: str) -> str:
    """
    Clean text by removing extra whitespace and normalizing
    """
    # Replace multiple whitespace with single space
    text = re.sub(r'\s+', ' ', text)
    # Remove extra newlines
    text = re.sub(r'\n\s*\n', '\n', text)
    # Strip leading/trailing whitespace
    return text.strip()

def extract_sections(text: str) -> List[Dict[str, Any]]:
    """
    Extract sections from text based on common heading patterns
    """
    sections = []

    # Look for common heading patterns
    heading_patterns = [
        r'^(#{1,6})\s+(.+)$',  # Markdown headings
        r'^(\d+[\.\d\s]+)(.+)$',  # Numbered sections like "1. Introduction"
        r'^([A-Z][A-Z\s]+)$',  # ALL CAPS headings
        r'^([A-Z][a-z].+):$',  # Title case headings ending with colon
    ]

    lines = text.split('\n')
    current_section = {"title": "Introduction", "content": "", "start_line": 0}

    for i, line in enumerate(lines):
        is_heading = False

        for pattern in heading_patterns:
            match = re.match(pattern, line.strip(), re.IGNORECASE)
            if match:
                # Save previous section if it has content
                if current_section["content"].strip():
                    sections.append(current_section)

                # Start new section
                current_section = {
                    "title": match.group(2).strip() if len(match.groups()) > 1 else match.group(0).strip(),
                    "content": "",
                    "start_line": i
                }
                is_heading = True
                break

        if not is_heading:
            current_section["content"] += line + "\n"

    # Add the last section if it has content
    if current_section["content"].strip():
        sections.append(current_section)

    return sections