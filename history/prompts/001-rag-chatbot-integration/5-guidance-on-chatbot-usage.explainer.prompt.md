---
id: 5
title: Guidance on Chatbot Usage
stage: explainer
date: 2025-12-25
surface: agent
model: gemini-2.0-flash-exp
feature: 001-rag-chatbot-integration
branch: null
user: Annas
command: null
labels: [chatbot, guidance, documentation]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files: []
tests: []
---

## Prompt

What I will right in chatbot? to get about book?

## Response snapshot

Since the book covers Physical AI and Humanoid Robotics, I suggested several topics based on the book's structure:
1. Core Concepts (Physical AI basics)
2. Robot Modeling (URDF)
3. Programming (rclpy/ROS 2)
4. Simulation (NVIDIA Isaac Sim)
5. Navigation (Nav2 Humanoid Planning)
I also highlighted the "Select & Ask" feature.

## Outcome

- ✅ Impact: Empowered the user to test the RAG chatbot with relevant queries.
- 🧪 Tests: n/a
- 📁 Files: none
- 🔁 Next prompts: none
- 🧠 Reflection: Providing concrete examples helps the user verify that the RAG (Retrieval-Augmented Generation) is working correctly by checking if the bot actually knows the book's unique content.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): n/a
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
