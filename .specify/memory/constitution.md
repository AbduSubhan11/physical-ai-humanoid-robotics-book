<!--
Version change: 0.0.0 → 1.0.0
List of modified principles:
- [PRINCIPLE_1_NAME] → Book Content Quality & Maintainability
- [PRINCIPLE_2_NAME] → RAG Chatbot Grounding & Safety
- [PRINCIPLE_3_NAME] → Claude Code + Spec-Kit Plus Workflow
- [PRINCIPLE_4_NAME] → Modularity, Scalability, Hackathon-Readiness
Added sections: Primary Deliverables
Removed sections: None
Templates requiring updates:
- .specify/templates/plan-template.md: ⚠ pending
- .specify/templates/spec-template.md: ⚠ pending
- .specify/templates/tasks-template.md: ⚠ pending
- .specify/templates/commands/*.md: ⚠ pending
Follow-up TODOs:
- TODO(GUIDANCE_FILE): Add explicit guidance file reference if applicable.
-->
# Physical AI & Humanoid Robotics: AI-Native Textbook + Integrated RAG Chatbot Constitution

## Core Principles

### I. Book Content Quality & Maintainability
The book content is written in a maintainable, spec-driven, high-quality way.

### II. RAG Chatbot Grounding & Safety
The RAG chatbot is grounded, safe, and only answers using book content.

### III. Claude Code + Spec-Kit Plus Workflow
Claude Code + Spec-Kit Plus remain the primary development workflow.

### IV. Modularity, Scalability, Hackathon-Readiness
The project remains modular, scalable, and hackathon-ready.

## Primary Deliverables

### AI-Native Book (Docusaurus)
- Fully written Physical AI & Humanoid Robotics textbook
- Structured into modules/chapters
- Stored in /frontend/docs
- Live on GitHub Pages
- Components:
  - ROS2
  - Digital Twin
  - Vision-Language-Action (VLA)
  - Humanoid robotics control
  - AI Brain architecture
  - Capstone project

### Integrated RAG Chatbot
- A retrieval-augmented chatbot embedded inside the book website.
- Tech Stack Requirements:
  - Frontend: JS + Chat UI widget (OpenAI ChatKit, or custom)
  - Backend: FastAPI (backend/app/main.py)
  - Vector DB: Qdrant Cloud (Free Tier)
  - SQL DB: Neon Serverless Postgres (auth + logs)
  - Model Provider: OpenAI or Claude (chat completion)
- Features:
  - Ask questions about the book
  - Grounded answers (no hallucinations)
  - “Answer using selected text only” mode
  - Support follow-up questions

## Governance
This Constitution defines the rules, quality standards, collaboration workflow, and decision-making principles for this project.

**Version**: 1.0.0 | **Ratified**: 2025-12-08 | **Last Amended**: 2025-12-08