---
id: chapter5-urdf-robot-description
title: "Chapter 5: URDF & Robot Description"
sidebar_label: "Ch 5: URDF & Robot Description"
description: "Defining the humanoid body using Unified Robot Description Format"
---

# Chapter 5: URDF & Robot Description

This chapter covers the Unified Robot Description Format (URDF) for defining robot models.

## Overview

In this chapter, we explore how to describe robots using URDF, the standard format for representing robot models in ROS.

## Topics Covered

- URDF basics and structure
- Link and joint definitions
- Visual and collision properties
- Materials and colors

## Key Concepts

:::info
**URDF** (Unified Robot Description Format) is an XML format for representing a robot model. It defines the robot's physical and visual properties, including kinematic and dynamic properties.
:::

## URDF Structure

A URDF file contains:
- Links: Rigid bodies of the robot
- Joints: Connections between links
- Transmissions: Components that describe how actuators connect to joints
- Gazebo tags: Extensions for simulation

## Example URDF

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <!-- Attachment link -->
  <link name="attachment_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.2"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
  </link>

  <!-- Joint connecting links -->
  <joint name="base_to_attachment" type="revolute">
    <parent link="base_link"/>
    <child link="attachment_link"/>
    <origin xyz="0.3 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1"/>
  </joint>
</robot>
```

## Best Practices

- Use consistent naming conventions for links and joints
- Define proper inertial properties for accurate simulation
- Separate complex robots into logical subassemblies

## Next Steps

Understanding URDF is essential for creating robot models for both simulation and real-world applications.

[Image: Diagram of URDF Robot Model Structure]