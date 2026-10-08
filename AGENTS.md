# AGENTS.md

Minimal pointer file for Codex and other agents. `CLAUDE.md` is authoritative; this file adds nothing to it.

1. Read `CLAUDE.md` (all of it, including the safety rules and path ownership).
2. Read `docs/agents/HANDOFF.md`.
3. ROS 2 is available through the project's container environment even though it is not installed on
   macOS. Use `./tools/dev/ros2_humble.sh build-test` (see `docs/agents/LOCAL_ROS2_BUILD_ENV.md`).
   Do not report ROS tests as unavailable without running it.
4. Run the relevant build and regression tests before committing.
5. Never push without the workflow's explicit authorization.
