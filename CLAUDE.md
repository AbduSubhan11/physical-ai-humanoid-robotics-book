roject: Physical AI & Humanoid Robotics — AI-Native Textbook

Built using Spec-Kit Plus + Claude Code
Deliverable for Hackathon 

1. 🧠 Purpose (Specifying the WHY)

Create a complete AI-Native Textbook that teaches the entire Physical AI & Humanoid Robotics course.
The project must delive/initr:

A Docusaurus-powered technical book, deployed on GitHub Pages.

A fully integrated RAG Chatbot, able to answer questions about the book with user-selected contextual grounding.

Backend using:

FastAPI

OpenAI Agents / ChatKit SDK

Neon Serverless Postgres

Qdrant Cloud Free Tier

Reusable Intelligence (50 bonus points):

Claude Code Subagents

Claude Code Skills

Automatic content generation agents

Auth + Personalization (50 bonus points):

Better-Auth signup/signin

Collect hardware/software background

Use user background to personalize the textbook chapters

Enable translation to Urdu (50 bonus points):

A button per chapter that rewrites content to Urdu

Uses backend API via FastAPI + OpenAI

This CLAUDE.md fully defines the project for Claude Code.

2. 📚 Deliverables (Spec-Kit Style)
Frontend (Docusaurus)

Home/Landing Page

Book Chapters (auto-generated via agents)

“Ask this chapter” RAG chatbot widget

Personalization button

Urdu translation button

SEO optimized

GitHub Pages deployment

Image optimization (webp)

Schema.org structured data for SEO

Backend (FastAPI + Agents + DB)

/rag/query → Answers based on embeddings (Qdrant)

/rag/selected → Answers based on user-selected text

/auth/signup → Better-Auth integration

/auth/signin

/personalize/chapter → Rewrites chapter using user’s background

/translate/urdu → Converts chapter to Urdu

/generate/chapter → Agent that creates chapter content

/health

Databases

Neon Postgres

users

backgrounds

personalization settings

Qdrant Cloud

embeddings of book chapters

embeddings of paragraphs for fine-grained lookup

Agents/Subagents

Book Chapter Generator Agent

SEO Optimizer Agent

RAG Query Agent

Translation Agent

Personalization Agent

Schema + Metadata Agent

Humanoid Robotics Tutor Agent

All agents must be usable within Claude Code as skills.

3. 🧱 Constitution Phase (Quality Standards)
Code Standards

Python 3.12

FastAPI + Pydantic

Docusaurus v3

TypeScript for frontend

Use uv for python env/packages

Black + Ruff formatting

Typed code (Pyright)

React components must be reusable

Architecture Rules

Backend in /backend

Frontend in /frontend

RAG + Agents code in /backend/ai

Document generation in /content

Documentation Rules

Each backend module must have a docstring

Each chapter must have frontmatter metadata

4. 📘 Specify Phase

(All features broken down into structured Spec-Kit deliverables)

4.1 Book Structure (Docusaurus)

Chapters generated from course:

Module 1 — The Robotic Nervous System (ROS 2)

ROS 2 Nodes

Topics & Services

rclpy example controllers

URDF for Humanoids

Module 2 — The Digital Twin (Gazebo & Unity)

Physics simulation

Sensor simulation

Depth cameras, LiDAR, IMU

Unity integration

Module 3 — The AI-Robot Brain (NVIDIA Isaac)

Isaac Sim

Isaac ROS

VSLAM

Nav2 for humanoid locomotion

Photorealistic synthetic data

Module 4 — Vision-Language-Action

Whisper → voice commands

GPT Planning → ROS 2 action sequences

Final Capstone

Additional Chapters

Hardware requirements

Workstation setup

Jetson Orin deployment

RealSense sensors

Unitree robots

Cloud vs Physical lab design

5. 🧬 Plan Phase (Architecture)
Final File Tree Claude Must Build
project-root/
  backend/
    app/
      api/
      models/
      services/
      database/
      ai/
      main.py
  frontend/
    docusaurus.config.js
    src/
    static/
    docs/
  content/
  agents/
  skills/
  README.md