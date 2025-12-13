from typing import List, Dict, Any
from ..models import BookContent
from sqlalchemy.orm import Session
import markdown
from bs4 import BeautifulSoup

class EducationalContentService:
    def __init__(self):
        pass

    def format_content_for_education(
        self,
        content: str,
        education_level: str = "intermediate",
        learning_objectives: List[str] = None
    ) -> Dict[str, Any]:
        """
        Format content specifically for educational purposes
        """
        formatted_content = {
            "original_content": content,
            "education_level": education_level,
            "learning_objectives": learning_objectives or [],
            "key_concepts": self._extract_key_concepts(content),
            "summary": self._generate_summary(content),
            "study_questions": self._generate_study_questions(content),
            "difficulty_level": self._determine_difficulty(content),
            "prerequisites": self._identify_prerequisites(content)
        }

        return formatted_content

    def _extract_key_concepts(self, content: str) -> List[str]:
        """
        Extract key concepts from the content
        """
        # Simple approach: look for bold text, headings, and important keywords
        soup = BeautifulSoup(content, 'html.parser')

        # Extract text that might be important (headings, bold, etc.)
        key_concepts = []

        # If content is plain text, look for common patterns
        lines = content.split('\n')
        for line in lines:
            line = line.strip()
            if line and (line.isupper() or line.startswith('#') or ':' in line or line.endswith(':')):
                # These might be headings or important terms
                key_concepts.append(line.replace('#', '').strip())

        # Remove duplicates while preserving order
        unique_concepts = []
        for concept in key_concepts:
            if concept not in unique_concepts and len(concept) > 5:  # Filter out very short items
                unique_concepts.append(concept)

        return unique_concepts[:10]  # Return top 10 concepts

    def _generate_summary(self, content: str) -> str:
        """
        Generate a summary of the content
        """
        # Simple approach: take the first few sentences
        sentences = content.split('.')
        if len(sentences) > 3:
            summary = '.'.join(sentences[:3]) + '.'
        else:
            summary = content[:200] + '...' if len(content) > 200 else content

        return summary

    def _generate_study_questions(self, content: str) -> List[str]:
        """
        Generate study questions based on the content
        """
        # Simple approach: look for question-like sentences or generate based on key concepts
        questions = []

        # Look for content that might contain questions
        lines = content.split('\n')
        for line in lines:
            if '?' in line and len(line) < 100:  # Likely a question
                questions.append(line.strip())

        # If no questions found, generate some based on key concepts
        if not questions:
            concepts = self._extract_key_concepts(content)[:3]
            for concept in concepts:
                questions.append(f"What is {concept}?")
                if len(questions) >= 3:
                    break

        return questions

    def _determine_difficulty(self, content: str) -> str:
        """
        Determine the difficulty level of the content
        """
        # Simple heuristic based on vocabulary complexity
        words = content.split()
        avg_word_length = sum(len(word) for word in words) / len(words) if words else 0

        if avg_word_length > 6:
            return "advanced"
        elif avg_word_length > 4:
            return "intermediate"
        else:
            return "beginner"

    def _identify_prerequisites(self, content: str) -> List[str]:
        """
        Identify prerequisites needed to understand the content
        """
        # Look for common prerequisite indicators
        prerequisites = []
        content_lower = content.lower()

        # Common robotics concepts that might be prerequisites
        common_prereqs = [
            "kinematics", "dynamics", "control theory", "sensors",
            "actuators", "programming", "mathematics", "physics"
        ]

        for prereq in common_prereqs:
            if prereq in content_lower:
                prerequisites.append(prereq.title())

        return list(set(prerequisites))  # Remove duplicates

    def organize_content_by_lessons(
        self,
        contents: List[Dict[str, Any]]
    ) -> List[Dict[str, Any]]:
        """
        Organize content into lessons based on educational structure
        """
        lessons = []

        for i, content in enumerate(contents):
            lesson = {
                "lesson_id": f"lesson_{i+1}",
                "title": content.get("title", f"Lesson {i+1}"),
                "content": content.get("content", ""),
                "duration_minutes": self._estimate_duration(content.get("content", "")),
                "difficulty": self._determine_difficulty(content.get("content", "")),
                "learning_objectives": content.get("learning_objectives", []),
                "key_concepts": self._extract_key_concepts(content.get("content", "")),
                "study_questions": self._generate_study_questions(content.get("content", ""))
            }
            lessons.append(lesson)

        return lessons

    def _estimate_duration(self, content: str) -> int:
        """
        Estimate reading duration in minutes based on content length
        """
        words_per_minute = 200  # Average reading speed
        word_count = len(content.split())
        minutes = word_count // words_per_minute
        return max(1, minutes)  # At least 1 minute