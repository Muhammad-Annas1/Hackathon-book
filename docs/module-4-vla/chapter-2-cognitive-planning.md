---
title: "Chapter 2: Cognitive Planning"
---

Cognitive planning is the "brain" of the autonomous robot. It involves taking a high-level, often abstract, goal and breaking it down into a concrete sequence of actions that the robot can execute. Large Language Models (LLMs) have emerged as a powerful tool for this task.

## LLMs for High-Level Planning

By leveraging their vast knowledge and reasoning capabilities, LLMs can act as a "common-sense" planner. Given a goal and a description of the robot's available actions, an LLM can generate a plausible sequence of steps to achieve that goal.

The process typically looks like this:

1.  **Prompt Engineering**: A carefully crafted prompt is sent to the LLM. This prompt includes:
    *   The high-level goal (e.g., "Get me a can of soda from the kitchen").
    *   A description of the robot's current state and environment.
    *   A list of the robot's primitive actions (e.g., `navigate_to(location)`, `find_object(object_name)`, `pick_up(object)`, `open(door)`).
2.  **LLM Inference**: The LLM processes the prompt and generates a response.
3.  **Output Parsing**: The LLM's response, which is typically a natural language text or a structured format like JSON, is parsed into a machine-readable list of actions.
4.  **Execution**: The robot's control system executes the actions in the sequence, one by one.

### Example: "Get a drink" Task

Here is an example of how an LLM can be prompted to generate a plan.

**System Prompt**:
```text
You are a helpful robot assistant. Your task is to take a high-level user command and break it down into a sequence of simple, executable actions.

Available actions:
- navigate_to(location)  // locations: kitchen, living_room, bedroom
- find_object(object_name)
- pick_up(object)
- place(object, location)
- open(object) // e.g., open(refrigerator)
- close(object) // e.g., close(refrigerator)

Return your plan as a JSON array of action objects.
```

**User Prompt**:
```text
"Please get me a can of soda from the kitchen."
```

**LLM-Generated Action Sequence (Output)**:

```json
[
  {
    "action": "navigate_to",
    "parameters": {
      "location": "kitchen"
    }
  },
  {
    "action": "open",
    "parameters": {
      "object": "refrigerator"
    }
  },
  {
    "action": "find_object",
    "parameters": {
      "object_name": "soda_can"
    }
  },
  {
    "action": "pick_up",
    "parameters": {
      "object": "soda_can"
    }
  },
  {
    "action": "close",
    "parameters": {
      "object": "refrigerator"
    }
  },
  {
    "action": "navigate_to",
    "parameters": {
      "location": "user_location"
    }
  },
  {
    "action": "place",
    "parameters": {
      "object": "soda_can",
      "location": "user_table"
    }
  }
]
```

This structured output can then be directly interpreted and executed by the robot's control system.