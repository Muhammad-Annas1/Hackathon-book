---

description: "Task list for feature implementation: Module 1: The Robotic Nervous System (ROS 2)"
---

# Tasks: Module 1: The Robotic Nervous System (ROS 2)

**Input**: Design documents from `/specs/001-module-1-ros2/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- Book content: `docs/module-1/`
- Code examples: `src/ros2/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure for the new module.

- [X] T001 Create module directory in `docs/module-1/`.

---

## Phase 2: User Story 1 - Understand Core ROS 2 Concepts (Priority: P1) 🎯 MVP

**Goal**: As a student, I want to understand the core concepts of ROS 2 (Nodes, Topics, Services), so that I can build a foundation for robotics programming.

**Independent Test**: The student can explain the purpose of ROS 2 Nodes, Topics, and Services and can use the ROS 2 command-line tools to inspect them.

### Implementation for User Story 1

- [X] T002 [US1] Create chapter file for Core ROS 2 Concepts in `docs/module-1/chapter-1-core-concepts.md`.
- [X] T003 [P] [US1] Write content explaining ROS 2 Nodes in `docs/module-1/chapter-1-core-concepts.md`.
- [X] T004 [P] [US1] Write content explaining ROS 2 Topics in `docs/module-1/chapter-1-core-concepts.md`.
- [X] T005 [P] [US1] Write content explaining ROS 2 Services in `docs/module-1/chapter-1-core-concepts.md`.
- [X] T006 [US1] Add runnable examples and CLI commands for Nodes, Topics, and Services in `docs/module-1/chapter-1-core-concepts.md`.

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently.

---

## Phase 3: User Story 2 - Control a Robot with rclpy (Priority: P2)

**Goal**: As a student, I want to learn how to control a robot using `rclpy`, so that I can write Python scripts to interact with a ROS 2 system.

**Independent Test**: The student can write a simple `rclpy` script to publish and subscribe to a topic.

### Implementation for User Story 2

- [X] T007 [US2] Create chapter file for `rclpy` control in `docs/module-1/chapter-2-rclpy-control.md`.
- [X] T008 [P] [US2] Create example `rclpy` publisher script in `src/ros2/examples/simple_publisher.py`.
- [X] T009 [P] [US2] Create example `rclpy` subscriber script in `src/ros2/examples/simple_subscriber.py`.
- [X] T010 [US2] Write content explaining `rclpy` and embedding the publisher/subscriber examples in `docs/module-1/chapter-2-rclpy-control.md`.

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently.

---

## Phase 4: User Story 3 - Describe a Humanoid Robot with URDF (Priority: P3)

**Goal**: As a student, I want to understand how to describe a humanoid robot's structure using URDF, so that I can model a robot for simulation and control.

**Independent Test**: The student can create a simple URDF file for a multi-joint robot and visualize it in RViz.

### Implementation for User Story 3

- [X] T011 [US3] Create chapter file for URDF fundamentals in `docs/module-1/chapter-3-urdf-fundamentals.md`.
- [X] T012 [P] [US3] Create a simple humanoid URDF file in `src/ros2/urdf/simple_humanoid.urdf`.
- [X] T013 [US3] Write content explaining URDF and how to visualize the model in RViz in `docs/module-1/chapter-3-urdf-fundamentals.md`.

**Checkpoint**: All user stories should now be independently functional.

---

## Phase 5: User Story 4 - Simple Humanoid Joint Control (Priority: P4, optional)

**Goal**: As a student, I want to build a simple humanoid joint control project, so that I can apply my knowledge of ROS 2, `rclpy`, and URDF.

**Independent Test**: The student can write a `rclpy` script to publish joint angles to a URDF model and see the model update in RViz.

### Implementation for User Story 4

- [X] T014 [US4] Create chapter file for the joint control project in `docs/module-1/chapter-4-joint-control-project.md`.
- [X] T015 [P] [US4] Create an `rclpy` script to publish `JointState` messages in `src/ros2/projects/joint_state_publisher.py`.
- [X] T016 [P] [US4] Create a ROS 2 launch file to start RViz and the joint state publisher in `src/ros2/launch/humanoid_control_launch.py`.
- [X] T017 [US4] Write content explaining the project and how to run it in `docs/module-1/chapter-4-joint-control-project.md`.

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories.

- [X] T018 [P] Review all markdown for Docusaurus compatibility and clarity in `docs/module-1/`.
- [X] T019 [P] Validate all code examples are runnable and correct in `src/ros2/`.
- [X] T020 Check for broken links within `docs/module-1/`.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately.
- **User Stories (Phase 2+)**: Depend on Setup completion.
  - User stories can then proceed in parallel or sequentially in priority order (P1 → P2 → P3 → P4).

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Setup. No dependencies on other stories.
- **User Story 2 (P2)**: Can start after Setup. Depends on concepts from US1.
- **User Story 3 (P3)**: Can start after Setup. Depends on concepts from US1.
- **User Story 4 (P4)**: Depends on the completion of US1, US2, and US3.

### Parallel Opportunities

- Once Setup is complete, work on US1, US2, and US3 can begin in parallel, though they build conceptually on each other.
- Tasks marked with `[P]` within each user story can be developed in parallel. For example, writing content for different sections or creating different example files.

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup.
2. Complete Phase 2: User Story 1.
3. **STOP and VALIDATE**: Review the content and examples for US1 independently.
4. Demo if ready.

### Incremental Delivery

1. Complete Setup.
2. Add User Story 1 → Review independently → Demo (MVP!).
3. Add User Story 2 → Review independently → Demo.
4. Add User Story 3 → Review independently → Demo.
5. Add User Story 4 → Review independently → Demo.
6. Each story adds a new layer of knowledge without breaking previous ones.
