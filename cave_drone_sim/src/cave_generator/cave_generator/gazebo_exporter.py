"""
gazebo_exporter.py
==================

GazeboExporter: serialises cave meshes and metadata into a Gazebo Harmonic
compatible model directory structure and world SDF file.

Output directory layout
-----------------------
::

    cave_model_001/
        model.config          ← Gazebo model metadata
        model.sdf             ← Model SDF with visual + collision geometry
        meshes/
            cave_visual.obj   ← High-poly visual mesh (OBJ + MTL)
            cave_collision.stl ← Decimated collision mesh

    cave_world.sdf            ← World file (includes model, lighting, physics)

Usage
-----
::

    exporter = GazeboExporter()
    model_dir = exporter.export_cave_model(
        visual_mesh, collision_mesh, "./output/cave_model_001"
    )
    exporter.export_world_file(
        model_dir,
        drone_spawn_pose=(0, 0, 1, 0, 0, 0),
        output_path="./output/cave_world.sdf",
    )
"""

from __future__ import annotations

import logging
import os
import textwrap
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import trimesh

logger = logging.getLogger(__name__)

# Default physics parameters for Gazebo Harmonic
_DEFAULT_PHYSICS = {
    "max_step_size": 0.001,
    "real_time_factor": 1.0,
    "real_time_update_rate": 1000,
    "gravity": "0 0 -9.81",
}


class GazeboExporter:
    """
    Export a cave mesh to a Gazebo Harmonic model and world SDF.

    Parameters
    ----------
    author_name : str
        Author string embedded in model.config.
    author_email : str
        Author e-mail embedded in model.config.
    """

    def __init__(
        self,
        author_name: str = "cave_generator",
        author_email: str = "cave@example.com",
    ) -> None:
        self.author_name = author_name
        self.author_email = author_email

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def export_cave_model(
        self,
        visual_mesh: trimesh.Trimesh,
        collision_mesh: trimesh.Trimesh,
        output_dir: str,
        model_name: str = "cave_model",
        model_description: str = "Procedurally generated cave for drone simulation.",
    ) -> str:
        """
        Write the full Gazebo model directory structure.

        Parameters
        ----------
        visual_mesh : trimesh.Trimesh
            High-resolution mesh for visual rendering.
        collision_mesh : trimesh.Trimesh
            Decimated mesh for physics simulation.
        output_dir : str
            Root directory for the model (will be created if absent).
        model_name : str
            Logical model name used inside SDF files.
        model_description : str
            Human-readable description embedded in model.config.

        Returns
        -------
        str
            Absolute path to the created model directory.
        """
        output_dir = os.path.abspath(output_dir)
        meshes_dir = os.path.join(output_dir, "meshes")
        os.makedirs(meshes_dir, exist_ok=True)

        # --- Export meshes ---
        visual_path = os.path.join(meshes_dir, "cave_visual.obj")
        collision_path = os.path.join(meshes_dir, "cave_collision.obj")

        visual_mesh.export(visual_path)
        logger.info("Wrote visual mesh: %s", visual_path)
        collision_mesh.export(collision_path, include_normals=True)
        logger.info("Wrote collision mesh: %s", collision_path)

        # --- model.config ---
        config_path = os.path.join(output_dir, "model.config")
        config_xml = self._make_model_config(model_name, model_description)
        _write_text(config_path, config_xml)

        # --- model.sdf ---
        sdf_path = os.path.join(output_dir, "model.sdf")
        model_sdf = self._make_model_sdf(model_name)
        _write_text(sdf_path, model_sdf)

        logger.info("Cave model exported to: %s", output_dir)
        return output_dir

    def export_world_file(
        self,
        cave_model_path: str,
        drone_spawn_pose: Tuple[float, ...] = (0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
        output_path: str = "cave_world.sdf",
        include_ground_plane: bool = True,
        point_light_positions: Optional[List[Tuple[float, float, float]]] = None,
        cave_z_offset: float = 0.0,
    ) -> str:
        """
        Generate a Gazebo Harmonic world SDF file.

        Parameters
        ----------
        cave_model_path : str
            Absolute path to the cave model directory (contains model.sdf).
        drone_spawn_pose : tuple[float, ...]
            (x, y, z, roll, pitch, yaw) spawn pose for the drone in metres/radians.
        output_path : str
            Output ``.sdf`` file path.
        include_ground_plane : bool
            If True, add a ground plane below the cave.
        point_light_positions : list of (x,y,z), optional
            Positions for point lights (cave entrance lighting).  If None,
            two lights are placed automatically near z_min.
        cave_z_offset : float
            Vertical offset applied to the cave model so its lowest point
            sits at the Gazebo ground plane (z=0).  Typically ``-mesh_min_z``.

        Returns
        -------
        str
            Absolute path to the written world file.
        """
        cave_model_path = os.path.abspath(cave_model_path)
        model_name = os.path.basename(cave_model_path)
        output_path = os.path.abspath(output_path)

        if point_light_positions is None:
            # Default: lights inside the cave, shifted up by the offset
            point_light_positions = [
                (-5.0, 0.0, cave_z_offset),
                (5.0, 0.0, cave_z_offset),
            ]

        world_sdf = self._make_world_sdf(
            cave_model_path=cave_model_path,
            model_name=model_name,
            drone_spawn_pose=drone_spawn_pose,
            include_ground_plane=include_ground_plane,
            point_light_positions=point_light_positions,
            cave_z_offset=cave_z_offset,
        )
        _write_text(output_path, world_sdf)
        logger.info("World SDF written to: %s", output_path)
        return output_path

    # ------------------------------------------------------------------
    # Private: XML generation
    # ------------------------------------------------------------------

    def _make_model_config(self, model_name: str, description: str) -> str:
        """Return model.config XML content."""
        return textwrap.dedent(f"""\
            <?xml version="1.0"?>
            <model>
              <name>{model_name}</name>
              <version>1.0</version>
              <sdf version="1.9">model.sdf</sdf>
              <author>
                <name>{self.author_name}</name>
                <email>{self.author_email}</email>
              </author>
              <description>
                {description}
              </description>
            </model>
        """)

    def _make_model_sdf(self, model_name: str) -> str:
        """Return model.sdf XML content for a static cave mesh."""
        return textwrap.dedent(f"""\
            <?xml version="1.0"?>
            <sdf version="1.9">
              <model name="{model_name}">
                <static>true</static>
                <link name="cave_link">

                  <!-- Visual geometry (high-poly OBJ) -->
                  <visual name="cave_visual">
                    <geometry>
                      <mesh>
                        <uri>model://{model_name}/meshes/cave_visual.obj</uri>
                        <scale>1 1 1</scale>
                      </mesh>
                    </geometry>
                    <material>
                      <ambient>0.2 0.2 0.2 1</ambient>
                      <diffuse>0.4 0.35 0.3 1</diffuse>
                      <specular>0.05 0.05 0.05 1</specular>
                      <double_sided>true</double_sided>
                    </material>
                  </visual>

                  <!-- Collision geometry (decimated OBJ) -->
                  <collision name="cave_collision">
                    <geometry>
                      <mesh>
                        <uri>model://{model_name}/meshes/cave_collision.obj</uri>
                        <scale>1 1 1</scale>
                      </mesh>
                    </geometry>
                    <surface>
                      <friction>
                        <ode>
                          <mu>0.8</mu>
                          <mu2>0.8</mu2>
                        </ode>
                      </friction>
                      <contact>
                        <ode>
                          <kp>1e6</kp>
                          <kd>1e2</kd>
                        </ode>
                      </contact>
                    </surface>
                  </collision>

                </link>
              </model>
            </sdf>
        """)

    def _make_world_sdf(
        self,
        cave_model_path: str,
        model_name: str,
        drone_spawn_pose: Tuple[float, ...],
        include_ground_plane: bool,
        point_light_positions: List[Tuple[float, float, float]],
        cave_z_offset: float = 0.0,
    ) -> str:
        """Return world SDF XML content."""
        px, py, pz = drone_spawn_pose[0], drone_spawn_pose[1], drone_spawn_pose[2]
        roll = drone_spawn_pose[3] if len(drone_spawn_pose) > 3 else 0.0
        pitch = drone_spawn_pose[4] if len(drone_spawn_pose) > 4 else 0.0
        yaw = drone_spawn_pose[5] if len(drone_spawn_pose) > 5 else 0.0

        # Ground plane XML
        ground_xml = ""
        if include_ground_plane:
            ground_xml = textwrap.dedent("""\
                    <!-- Ground plane below the cave -->
                    <model name="ground_plane">
                      <static>true</static>
                      <link name="ground_link">
                        <collision name="ground_collision">
                          <geometry>
                            <plane>
                              <normal>0 0 1</normal>
                              <size>200 200</size>
                            </plane>
                          </geometry>
                        </collision>
                        <visual name="ground_visual">
                          <geometry>
                            <plane>
                              <normal>0 0 1</normal>
                              <size>200 200</size>
                            </plane>
                          </geometry>
                          <material>
                            <ambient>0.15 0.12 0.1 1</ambient>
                            <diffuse>0.3 0.25 0.2 1</diffuse>
                          </material>
                        </visual>
                        <pose>0 0 0 0 0 0</pose>
                      </link>
                    </model>
            """)

        # Point lights XML
        lights_xml_parts: List[str] = []
        for idx, (lx, ly, lz) in enumerate(point_light_positions):
            lights_xml_parts.append(textwrap.dedent(f"""\
                    <light name="point_light_{idx}" type="point">
                      <pose>{lx} {ly} {lz} 0 0 0</pose>
                      <diffuse>0.7 0.65 0.55 1</diffuse>
                      <specular>0.1 0.1 0.1 1</specular>
                      <attenuation>
                        <range>25</range>
                        <constant>0.4</constant>
                        <linear>0.05</linear>
                        <quadratic>0.001</quadratic>
                      </attenuation>
                      <cast_shadows>false</cast_shadows>
                    </light>
            """))
        lights_xml = "\n".join(lights_xml_parts)

        return textwrap.dedent(f"""\
            <?xml version="1.0"?>
            <sdf version="1.9">
              <world name="cave_world">

                <!-- Physics engine configuration -->
                <physics name="default_physics" default="true" type="ode">
                  <max_step_size>{_DEFAULT_PHYSICS["max_step_size"]}</max_step_size>
                  <real_time_factor>{_DEFAULT_PHYSICS["real_time_factor"]}</real_time_factor>
                  <real_time_update_rate>{_DEFAULT_PHYSICS["real_time_update_rate"]}</real_time_update_rate>
                </physics>

                <!-- Gravity must be a direct child of <world>, not <physics> -->
                <gravity>{_DEFAULT_PHYSICS["gravity"]}</gravity>

                <!-- Magnetic field (required for PX4 magnetometer) -->
                <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
                <atmosphere type="adiabatic"/>

                <!-- GPS reference frame (required for NavSat) -->
                <spherical_coordinates>
                  <surface_model>EARTH_WGS84</surface_model>
                  <world_frame_orientation>ENU</world_frame_orientation>
                  <latitude_deg>47.397971057728974</latitude_deg>
                  <longitude_deg>8.546163739800146</longitude_deg>
                  <elevation>0</elevation>
                </spherical_coordinates>

                <!-- System plugins (from PX4 server.config) -->
                <plugin filename="gz-sim-physics-system"
                        name="gz::sim::systems::Physics">
                </plugin>
                <plugin filename="gz-sim-user-commands-system"
                        name="gz::sim::systems::UserCommands">
                </plugin>
                <plugin filename="gz-sim-scene-broadcaster-system"
                        name="gz::sim::systems::SceneBroadcaster">
                </plugin>
                <plugin filename="gz-sim-contact-system"
                        name="gz::sim::systems::Contact">
                </plugin>
                <plugin filename="gz-sim-imu-system"
                        name="gz::sim::systems::Imu">
                </plugin>
                <plugin filename="gz-sim-air-pressure-system"
                        name="gz::sim::systems::AirPressure">
                </plugin>
                <plugin filename="gz-sim-air-speed-system"
                        name="gz::sim::systems::AirSpeed">
                </plugin>
                <plugin filename="gz-sim-apply-link-wrench-system"
                        name="gz::sim::systems::ApplyLinkWrench">
                </plugin>
                <plugin filename="gz-sim-navsat-system"
                        name="gz::sim::systems::NavSat">
                </plugin>
                <plugin filename="gz-sim-magnetometer-system"
                        name="gz::sim::systems::Magnetometer">
                </plugin>
                <plugin filename="gz-sim-sensors-system"
                        name="gz::sim::systems::Sensors">
                  <render_engine>ogre2</render_engine>
                </plugin>

                <!-- Scene ambient lighting (dim for cave atmosphere) -->
                <scene>
                  <ambient>0.05 0.05 0.08 1</ambient>
                  <background>0.0 0.0 0.0 1</background>
                  <shadows>false</shadows>
                </scene>

                <!-- Directional sun (weak, to simulate faint surface light) -->
                <light name="sun" type="directional">
                  <pose>0 0 10 0 0 0</pose>
                  <diffuse>0.1 0.1 0.15 1</diffuse>
                  <specular>0.02 0.02 0.02 1</specular>
                  <direction>0.1 0.1 -1</direction>
                  <cast_shadows>false</cast_shadows>
                </light>

                {lights_xml}

                <!-- Cave model (lifted so lowest point is at ground plane z=0) -->
                <include>
                  <uri>{cave_model_path}</uri>
                  <name>{model_name}</name>
                  <pose>0 0 {cave_z_offset:.4f} 0 0 0</pose>
                </include>

                {ground_xml}

                <!-- Drone spawn point marker (static invisible box) -->
                <model name="drone_spawn_point">
                  <static>true</static>
                  <pose>{px} {py} {pz} {roll} {pitch} {yaw}</pose>
                  <link name="spawn_link">
                    <visual name="spawn_visual">
                      <geometry>
                        <box><size>0.1 0.1 0.1</size></box>
                      </geometry>
                      <transparency>1.0</transparency>
                    </visual>
                  </link>
                </model>

              </world>
            </sdf>
        """)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _write_text(path: str, content: str) -> None:
    """Write *content* to *path*, creating parent directories as needed."""
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(content)
    logger.debug("Wrote: %s", path)
