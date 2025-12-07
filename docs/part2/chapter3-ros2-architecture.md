---
id: chapter3-ros2-architecture
title: "Chapter 3: ROS 2 Architecture"
sidebar_label: "Ch 3: ROS 2 Architecture"
description: "Understanding Nodes, Topics, and Services in ROS 2"
---

# Chapter 3: ROS 2 Architecture

This chapter provides an in-depth look at the core architectural concepts of ROS 2.

## Overview

In this chapter, we explore the fundamental building blocks of ROS 2: nodes, topics, and services.

## Topics Covered

- ROS 2 Nodes and their lifecycle
- Topics and message passing
- Services for request-response communication
- Parameters and configuration

## Core Concepts

:::info
**ROS 2** (Robot Operating System 2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.
:::

## Architecture Components

### Nodes
Nodes are the fundamental building blocks of ROS 2 applications. Each node is a process that performs computation.

### Topics
Topics enable asynchronous message passing between nodes using a publish/subscribe pattern.

### Services
Services enable synchronous request/response communication between nodes.

## Example Code

```python
# Example ROS 2 Node structure
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World'
        self.publisher.publish(msg)
```

## Next Steps

Understanding ROS 2 architecture is crucial for developing distributed robotics applications.

[Image: Diagram of ROS 2 Architecture Components]