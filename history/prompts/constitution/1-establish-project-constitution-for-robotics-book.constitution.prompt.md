---
id: 1
title: Establish Project Constitution for Robotics Book
stage: constitution
date: 2025-12-07
surface: agent
model: Gemini
feature: none
branch: master
user: Muhammad Annas
command: /sp.constitution
labels: [ "constitution", "project-setup" ]
links:
  spec: "null"
  ticket: "null"
  adr: "null"
  pr: "null"
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

/sp.constitution /sp.constitution

Project: Book + RAG Chatbot on Physical AI & Humanoid Robotics

Core principles:
- High technical accuracy (ROS 2, Gazebo, Unity, NVIDIA Isaac, VLA)
- Clarity for CS/AI/robotics students
- Consistent with official docs and verifiable code
- Modular, reproducible, Docusaurus-ready content

Key standards:
- Claims must be source-verified (official docs or research papers)
- Citation style: IEEE
- Min 40% robotics/AI research or official documentation sources
- All RAG examples reproducible using OpenAI Agents/ChatKit, FastAPI, Qdrant, Neon

Constraints:
- Book length: 8–12 chapters (~25k–35k words)
- Docusaurus-friendly Markdown
- GitHub Pages deployment required
- Includes a working RAG chatbot (embedding → Qdrant → FastAPI → ChatKit)
- Use Spec-Kit Plus + Claude Code for generation

Success criteria:
- Docusaurus build passes with no errors
- Book deployed on GitHub Pages
- RAG chatbot answers only from book content
- Accurate, non-hallucinated robotics explanations
- Verified code for ROS 2, Gazebo, Isaac, and VLA pipelines

## Response snapshot

I have updated the project constitution in `.specify/memory/constitution.md` based on your input. The new version is v1.0.0. I have also verified that the other template files do not need any changes.

## Outcome

- ✅ Impact: The project constitution was established, defining core principles and standards.
- 🧪 Tests: none
- 📁 Files: .specify/memory/constitution.md
- 🔁 Next prompts: none
- 🧠 Reflection: The bash script for creating PHR was missing, so I had to fall back to the agent-native method.

## Evaluation notes (flywheel)

- Failure modes observed: The `create-phr.sh` script was not found.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): "null"
- Next experiment (smallest change to try): Ensure the `create-phr.sh` script exists or update the documentation to reflect the correct script path.
