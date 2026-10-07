"""Minimal stand-in for ROS1 genmsg, only what px_generate_uorb_topic_helper's hash code calls.

The hash functions under test are the firmware's own, unmodified; this shim only supplies the
*parsing* (fields in file order, constants excluded, nested types resolved to <pkg>/<Type>).
"""
from . import msg_loader, msgs, names  # noqa: F401
