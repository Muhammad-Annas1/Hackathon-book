---
title: "Chapter 2: Cognitive Planning with LLMs"
sidebar_position: 2
---

Cognitive planning is the "brain" of an autonomous robot. It involves taking a high-level, often abstract, goal and breaking it down into a concrete sequence of actions.

## LLMs for High-Level Planning

By leveraging their vast knowledge and reasoning capabilities, Large Language Models (LLMs) can act as a "common-sense" planner. Given a goal and a description of the robot's available actions, an LLM can generate a plausible sequence of steps.

### The Planning Loop
1.  **Prompting**: The goal, the robot's state, and available primitive actions are sent to the LLM.
2.  **Inference**: The LLM reasons about the task (e.g., "To get a soda, I first need to be in the kitchen").
3.  **Output Parsing**: The model's response is converted into a machine-readable format like JSON.
4.  **Execution**: The robot's control system executes the sequence.

---

### Example: "Get a drink" Task

**System Prompt (Simplifed)**:
```text
You are a humanoid robot. Available actions:
- navigate_to(location)
- find_object(object_name)
- pick_up(object)
- open(object)
Return a JSON array of actions.
```

**User Goal**:
> "Please get me a can of soda from the kitchen."

**LLM-Generated Plan**:

```json
[
  { "action": "navigate_to", "params": { "location": "kitchen" } },
  { "action": "open", "params": { "object": "fridge" } },
  { "action": "find_object", "params": { "name": "soda_can" } },
  { "action": "pick_up", "params": { "object": "soda_can" } }
]
```

## Moving Beyond Fixed Plans
Modern cognitive planning also involves **Dynamic Re-planning**. If the robot finds the fridge is empty, it should be able to report back to the user or look for a soda in another location without needing a new manual command.