from typing import List, Dict, Any
import re

def chunk_content(text: str, chunk_size: int = 1000, overlap: int = 100) -> List[Dict[str, Any]]:
    """
    Split content into overlapping chunks of specified size with metadata
    """
    if len(text) <= chunk_size:
        return [{
            "text": text,
            "start_pos": 0,
            "end_pos": len(text),
            "chunk_index": 0,
            "size": len(text)
        }]

    chunks = []
    start = 0
    chunk_index = 0

    while start < len(text):
        end = start + chunk_size

        # If this is not the last chunk, try to break at sentence boundary
        if end < len(text):
            # Look for sentence endings near the end
            sentence_end = max(
                text.rfind('.', start, end),
                text.rfind('!', start, end),
                text.rfind('?', start, end)
            )

            if sentence_end > start + chunk_size // 2:  # Only if sentence end is reasonably close
                end = sentence_end + 1
            else:
                # If no sentence end found, break at word boundary
                word_end = text.rfind(' ', start + chunk_size // 2, end)
                if word_end > start + chunk_size // 2:
                    end = word_end

        chunk_text = text[start:end].strip()

        chunks.append({
            "text": chunk_text,
            "start_pos": start,
            "end_pos": end,
            "chunk_index": chunk_index,
            "size": len(chunk_text)
        })

        chunk_index += 1

        # Move start position with overlap
        start = end - overlap if end < len(text) else len(text)

        # Adjust start to avoid empty chunks
        if start >= len(text):
            break

        # Ensure we don't create overlapping chunks that are too small
        if end - start < 50 and start < len(text):
            start = end

    # Filter out empty chunks
    return [chunk for chunk in chunks if chunk["size"] > 0]

def chunk_by_semantic_boundaries(text: str, max_chunk_size: int = 1000) -> List[Dict[str, Any]]:
    """
    Split content by semantic boundaries like paragraphs, sections, etc.
    """
    # Split by paragraphs first
    paragraphs = text.split('\n\n')

    chunks = []
    current_chunk = ""
    current_start_pos = 0
    chunk_index = 0

    for para in paragraphs:
        # If adding this paragraph would exceed the max size
        if len(current_chunk) + len(para) > max_chunk_size and current_chunk:
            # Save the current chunk
            chunks.append({
                "text": current_chunk.strip(),
                "start_pos": current_start_pos,
                "end_pos": current_start_pos + len(current_chunk),
                "chunk_index": chunk_index,
                "size": len(current_chunk)
            })

            # Start a new chunk with this paragraph
            current_chunk = para
            current_start_pos = current_start_pos + len(current_chunk) - len(para)
            chunk_index += 1
        else:
            # Add this paragraph to current chunk
            if current_chunk:
                current_chunk += "\n\n" + para
            else:
                current_chunk = para

    # Add the last chunk if it has content
    if current_chunk.strip():
        chunks.append({
            "text": current_chunk.strip(),
            "start_pos": current_start_pos,
            "end_pos": current_start_pos + len(current_chunk),
            "chunk_index": chunk_index,
            "size": len(current_chunk)
        })

    return chunks

def chunk_by_headings(text: str, max_chunk_size: int = 2000) -> List[Dict[str, Any]]:
    """
    Split content by headings, keeping content under each heading together
    """
    # Regular expressions for different heading patterns
    heading_patterns = [
        r'^(#{1,6})\s+(.+)$',  # Markdown headings
        r'^(\d+[\.\d\s]+)(.+)$',  # Numbered sections like "1. Introduction"
        r'^([A-Z][A-Z\s]+)$',  # ALL CAPS headings
        r'^([A-Z][a-z].+):$',  # Title case headings ending with colon
    ]

    lines = text.split('\n')
    chunks = []
    current_chunk = {"title": "Introduction", "content": "", "start_line": 0, "chunk_index": 0}

    for i, line in enumerate(lines):
        is_heading = False

        for pattern in heading_patterns:
            match = re.match(pattern, line.strip(), re.IGNORECASE)
            if match:
                # If current chunk has content, save it before starting new one
                if current_chunk["content"].strip():
                    full_text = f"{current_chunk['title']}\n\n{current_chunk['content']}".strip()
                    if len(full_text) > max_chunk_size:
                        # If the chunk is too large, split it further
                        sub_chunks = chunk_content(full_text, max_chunk_size, 100)
                        for sub_chunk in sub_chunks:
                            chunks.append({
                                "text": sub_chunk["text"],
                                "start_pos": sub_chunk["start_pos"],
                                "end_pos": sub_chunk["end_pos"],
                                "chunk_index": len(chunks),
                                "size": sub_chunk["size"],
                                "title": current_chunk["title"]
                            })
                    else:
                        chunks.append({
                            "text": full_text,
                            "start_pos": current_chunk["start_pos"] if hasattr(current_chunk, 'start_pos') else i,
                            "end_pos": i,
                            "chunk_index": len(chunks),
                            "size": len(full_text),
                            "title": current_chunk["title"]
                        })

                # Start new chunk
                current_chunk = {
                    "title": match.group(2).strip() if len(match.groups()) > 1 else match.group(0).strip(),
                    "content": "",
                    "start_line": i,
                    "chunk_index": len(chunks)
                }
                is_heading = True
                break

        if not is_heading:
            current_chunk["content"] += line + "\n"

    # Add the last section if it has content
    if current_chunk["content"].strip():
        full_text = f"{current_chunk['title']}\n\n{current_chunk['content']}".strip()
        if len(full_text) > max_chunk_size:
            # If the chunk is too large, split it further
            sub_chunks = chunk_content(full_text, max_chunk_size, 100)
            for sub_chunk in sub_chunks:
                chunks.append({
                    "text": sub_chunk["text"],
                    "start_pos": sub_chunk["start_pos"],
                    "end_pos": sub_chunk["end_pos"],
                    "chunk_index": len(chunks),
                    "size": sub_chunk["size"],
                    "title": current_chunk["title"]
                })
        else:
            chunks.append({
                "text": full_text,
                "start_pos": current_chunk["start_pos"] if hasattr(current_chunk, 'start_pos') else 0,
                "end_pos": len(full_text),
                "chunk_index": len(chunks),
                "size": len(full_text),
                "title": current_chunk["title"]
            })

    return chunks