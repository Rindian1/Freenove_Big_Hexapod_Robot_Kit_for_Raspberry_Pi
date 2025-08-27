"""
Gait Pattern Implementations for Hexapod Robot

This module contains implementations of various gait patterns for hexapod locomotion.
"""

import math
import asyncio
from typing import Dict, List, Tuple, Optional, Any
from enum import Enum, auto
import logging

# Configure logging
logger = logging.getLogger(__name__)

class LegPosition(Enum):
    """Positions of the hexapod's legs."""
    FRONT_RIGHT = 0
    MIDDLE_RIGHT = 1
    REAR_RIGHT = 2
    REAR_LEFT = 3
    MIDDLE_LEFT = 4
    FRONT_LEFT = 5

class GaitState(Enum):
    """States of a leg during gait cycle."""
    SWING = auto()     # Leg is in the air, moving forward
    STANCE = auto()    # Leg is on the ground, pushing backward
    LIFT = auto()      # Leg is lifting off the ground
    LOWER = auto()     # Leg is lowering to the ground

class GaitController:
    """Controls the gait patterns for a hexapod robot."""
    
    def __init__(self, hexapod_controller):
        """Initialize the gait controller.
        
        Args:
            hexapod_controller: Reference to the main hexapod controller
        """
        self.hexapod = hexapod_controller
        self.cycle_time = 1.0  # Default cycle time in seconds
        self.duty_factor = 0.5  # Percentage of time in stance phase
        self.swing_height = 30  # Height of leg swing in mm
        self.step_length = 50   # Length of each step in mm
        
        # Gait state
        self.cycle_progress = 0.0  # 0.0 to 1.0
        self.leg_states = {
            leg: GaitState.STANCE for leg in LegPosition
        }
        self.leg_phases = {
            LegPosition.FRONT_RIGHT: 0.0,
            LegPosition.MIDDLE_RIGHT: 0.5,
            LegPosition.REAR_RIGHT: 0.0,
            LegPosition.REAR_LEFT: 0.5,
            LegPosition.MIDDLE_LEFT: 0.0,
            LegPosition.FRONT_LEFT: 0.5
        }
        
        # Movement parameters
        self.current_direction = 0.0  # Radians, 0 is forward
        self.current_rotation = 0.0   # Radians per second
        self.current_speed = 0.0      # 0.0 to 1.0
        
        # Threading
        self._running = False
        self._update_task = None
    
    async def start(self) -> None:
        """Start the gait controller update loop."""
        if self._running:
            return
            
        self._running = True
        self._update_task = asyncio.create_task(self._update_loop())
    
    async def stop(self) -> None:
        """Stop the gait controller."""
        self._running = False
        if self._update_task:
            self._update_task.cancel()
            try:
                await self._update_task
            except asyncio.CancelledError:
                pass
            self._update_task = None
    
    async def _update_loop(self) -> None:
        """Main update loop for the gait controller."""
        last_time = asyncio.get_event_loop().time()
        
        while self._running:
            # Calculate delta time
            current_time = asyncio.get_event_loop().time()
            dt = current_time - last_time
            last_time = current_time
            
            # Update cycle progress
            self.cycle_progress = (self.cycle_progress + (dt / self.cycle_time)) % 1.0
            
            # Update leg positions
            await self._update_leg_positions()
            
            # Small sleep to prevent busy-waiting
            await asyncio.sleep(0.01)
    
    async def _update_leg_positions(self) -> None:
        """Update the position of all legs based on the current gait."""
        for leg in LegPosition:
            phase = (self.cycle_progress + self.leg_phases[leg]) % 1.0
            
            # Determine leg state based on phase
            if phase < self.duty_factor:
                # Stance phase (leg on ground, pushing backward)
                state = GaitState.STANCE
                progress = phase / self.duty_factor  # 0.0 to 1.0
            else:
                # Swing phase (leg in air, moving forward)
                state = GaitState.SWING
                progress = (phase - self.duty_factor) / (1.0 - self.duty_factor)  # 0.0 to 1.0
            
            # Update leg state
            old_state = self.leg_states[leg]
            self.leg_states[leg] = state
            
            # Handle state transitions
            if old_state != state:
                if state == GaitState.STANCE:
                    await self._on_leg_stance_start(leg)
                elif state == GaitState.SWING:
                    await self._on_leg_swing_start(leg)
            
            # Calculate target position
            target_pos = self._calculate_leg_position(leg, state, progress)
            
            # Move leg to target position
            await self._move_leg(leg, target_pos)
    
    def _calculate_leg_position(self, leg: LegPosition, state: GaitState, progress: float) -> Tuple[float, float, float]:
        """Calculate the target position for a leg.
        
        Args:
            leg: The leg to calculate position for
            state: Current gait state of the leg
            progress: Progress through the current state (0.0 to 1.0)
            
        Returns:
            Tuple[x, y, z] target position in leg coordinate space
        """
        # Default position (leg extended downward)
        x, y, z = 0.0, 0.0, -100.0  # mm
        
        if state == GaitState.STANCE:
            # Leg is on the ground, pushing backward
            x = -self.step_length * (1.0 - progress)
            y = 0.0
            z = 0.0  # On the ground
        else:  # SWING
            # Leg is in the air, moving forward
            x = self.step_length * progress
            
            # Calculate parabolic height for smooth swing
            height = 4.0 * self.swing_height * progress * (1.0 - progress)
            z = -height
        
        # Apply rotation if needed
        if self.current_rotation != 0.0:
            # TODO: Implement rotation
            pass
        
        return x, y, z
    
    async def _move_leg(self, leg: LegPosition, position: Tuple[float, float, float]) -> None:
        """Move a leg to the specified position.
        
        Args:
            leg: The leg to move
            position: Target position (x, y, z) in leg coordinate space
        """
        if hasattr(self.hexapod, 'set_leg_position'):
            await self.hexapod.set_leg_position(leg, position)
    
    async def _on_leg_swing_start(self, leg: LegPosition) -> None:
        """Called when a leg starts swinging."""
        logger.debug(f"{leg.name} starting swing phase")
        
    async def _on_leg_stance_start(self, leg: LegPosition) -> None:
        """Called when a leg starts stance phase."""
        logger.debug(f"{leg.name} starting stance phase")
    
    # Gait pattern implementations
    
    async def set_tripod_gait(self, speed: float = 100.0, step_length: float = 50.0, 
                            step_height: float = 30.0, cycle_time: float = 1.0) -> None:
        """Configure and enable tripod gait.
        
        Args:
            speed: Movement speed (0-100%)
            step_length: Length of each step (mm)
            step_height: Height of leg swing (mm)
            cycle_time: Time for one complete gait cycle (seconds)
        """
        self.cycle_time = max(0.1, cycle_time * (100.0 / max(1.0, speed)))
        self.step_length = step_length
        self.swing_height = step_height
        self.duty_factor = 0.5  # 50% stance, 50% swing
        
        # Set leg phases for tripod gait
        self.leg_phases = {
            LegPosition.FRONT_RIGHT: 0.0,
            LegPosition.MIDDLE_RIGHT: 0.5,
            LegPosition.REAR_RIGHT: 0.0,
            LegPosition.REAR_LEFT: 0.5,
            LegPosition.MIDDLE_LEFT: 0.0,
            LegPosition.FRONT_LEFT: 0.5
        }
        
        logger.info(f"Tripod gait configured: {step_length}mm step, {step_height}mm height, {self.cycle_time:.2f}s cycle")
    
    async def set_wave_gait(self, speed: float = 100.0, step_length: float = 40.0, 
                          step_height: float = 30.0, cycle_time: float = 1.0) -> None:
        """Configure and enable wave gait.
        
        Args:
            speed: Movement speed (0-100%)
            step_length: Length of each step (mm)
            step_height: Height of leg swing (mm)
            cycle_time: Time for one complete gait cycle (seconds)
        """
        self.cycle_time = max(0.1, cycle_time * (100.0 / max(1.0, speed)))
        self.step_length = step_length
        self.swing_height = step_height
        self.duty_factor = 0.8  # 80% stance, 20% swing (one leg at a time)
        
        # Set leg phases for wave gait (one leg swings at a time)
        phase_step = 1.0 / 6.0
        self.leg_phases = {
            LegPosition.FRONT_RIGHT: 0 * phase_step,
            LegPosition.MIDDLE_RIGHT: 1 * phase_step,
            LegPosition.REAR_RIGHT: 2 * phase_step,
            LegPosition.REAR_LEFT: 3 * phase_step,
            LegPosition.MIDDLE_LEFT: 4 * phase_step,
            LegPosition.FRONT_LEFT: 5 * phase_step
        }
        
        logger.info(f"Wave gait configured: {step_length}mm step, {step_height}mm height, {self.cycle_time:.2f}s cycle")
    
    async def set_ripple_gait(self, speed: float = 100.0, step_length: float = 50.0, 
                            step_height: float = 30.0, cycle_time: float = 1.0) -> None:
        """Configure and enable ripple gait.
        
        Args:
            speed: Movement speed (0-100%)
            step_length: Length of each step (mm)
            step_height: Height of leg swing (mm)
            cycle_time: Time for one complete gait cycle (seconds)
        """
        self.cycle_time = max(0.1, cycle_time * (100.0 / max(1.0, speed)))
        self.step_length = step_length
        self.swing_height = step_height
        self.duty_factor = 0.75  # 75% stance, 25% swing (two legs at a time)
        
        # Set leg phases for ripple gait (two legs swing at a time, opposite sides)
        phase_step = 0.25
        self.leg_phases = {
            LegPosition.FRONT_RIGHT: 0 * phase_step,
            LegPosition.REAR_LEFT: 0 * phase_step,
            LegPosition.MIDDLE_RIGHT: 2 * phase_step,
            LegPosition.MIDDLE_LEFT: 2 * phase_step,
            LegPosition.REAR_RIGHT: 1 * phase_step,
            LegPosition.FRONT_LEFT: 1 * phase_step
        }
        
        logger.info(f"Ripple gait configured: {step_length}mm step, {step_height}mm height, {self.cycle_time:.2f}s cycle")
    
    # Movement control
    
    async def move_forward(self, speed: float = 100.0) -> None:
        """Move the hexapod forward.
        
        Args:
            speed: Movement speed (0-100%)
        """
        self.current_direction = 0.0  # 0 radians is forward
        self.current_speed = max(0.0, min(1.0, speed / 100.0))
        await self.set_tripod_gait(speed)
        
        if not self._running:
            await self.start()
    
    async def move_backward(self, speed: float = 100.0) -> None:
        """Move the hexapod backward.
        
        Args:
            speed: Movement speed (0-100%)
        """
        self.current_direction = math.pi  # 180 degrees is backward
        self.current_speed = max(0.0, min(1.0, speed / 100.0))
        await self.set_tripod_gait(speed)
        
        if not self._running:
            await self.start()
    
    async def turn_left(self, speed: float = 100.0) -> None:
        """Turn the hexapod left in place.
        
        Args:
            speed: Rotation speed (0-100%)
        """
        self.current_rotation = math.radians(30)  # 30 degrees per second
        self.current_speed = max(0.0, min(1.0, speed / 100.0))
        
        # Adjust step length for rotation
        step_length = 30.0  # Shorter steps for rotation
        await self.set_tripod_gait(speed, step_length=step_length)
        
        if not self._running:
            await self.start()
    
    async def turn_right(self, speed: float = 100.0) -> None:
        """Turn the hexapod right in place.
        
        Args:
            speed: Rotation speed (0-100%)
        """
        self.current_rotation = -math.radians(30)  # -30 degrees per second
        self.current_speed = max(0.0, min(1.0, speed / 100.0))
        
        # Adjust step length for rotation
        step_length = 30.0  # Shorter steps for rotation
        await self.set_tripod_gait(speed, step_length=step_length)
        
        if not self._running:
            await self.start()
    
    async def stop(self) -> None:
        """Stop all movement."""
        self.current_speed = 0.0
        self.current_rotation = 0.0
        await super().stop()
