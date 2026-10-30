"""Frequently changing robot state (pose, path, health reports) kept in memory instead of the DB. Assumes a single server process.

A separate module because consumers and services/planning both use it.
"""
LIVE = {}  # robot_name -> {'pose': {...}, 'path': [...], 'health': {...}, 'mode': ...}
