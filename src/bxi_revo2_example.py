#!/usr/bin/env python3
"""Revo2 BxiPci CANFD Python demo matching bxi_revo2_example.cpp."""

from __future__ import annotations

import asyncio
from typing import Optional

import rclpy

from bxi_can_node import (
    BxiDeviceContext,
    cleanup_bxipci_device,
    init_bxipci_device,
    libstark,
    stop_bxipci_runtime,
)


async def print_device_info(ctx: BxiDeviceContext) -> None:
    info = await ctx.handle.get_device_info(ctx.slave_id)
    print("Device Info:")
    print(f"  Hardware Type: {info.hardware_type}")
    print(f"  Serial Number: {info.serial_number or ''}")
    print(f"  Firmware Version: {info.firmware_version or ''}")


async def print_motor_status(ctx: BxiDeviceContext) -> None:
    print("[Demo] Reading motor status...")
    status = await ctx.handle.get_motor_status(ctx.slave_id)
    print(f"  Positions: {list(status.positions)}")
    print(f"  Speeds: {list(status.speeds)}")
    print(f"  Currents: {list(status.currents)}")
    print(f"  States: {list(status.states)}")


async def demo_basic_position(ctx: BxiDeviceContext) -> None:
    print("\n=== Demo 1: Basic Position Control ===")
    delay = 1.0

    print("[Demo] Performing fist gesture...")
    await ctx.handle.set_finger_positions(ctx.slave_id, [500, 500, 1000, 1000, 1000, 1000])
    await asyncio.sleep(delay)

    print("[Demo] Performing open hand...")
    await ctx.handle.set_finger_positions(ctx.slave_id, [0, 0, 0, 0, 0, 0])
    await asyncio.sleep(delay)

    print("[Demo] Moving single finger (middle)...")
    await ctx.handle.set_finger_position(ctx.slave_id, libstark.FingerId.Middle, 800)
    await asyncio.sleep(delay)
    await ctx.handle.set_finger_position(ctx.slave_id, libstark.FingerId.Middle, 0)
    await asyncio.sleep(delay)

    await print_motor_status(ctx)


async def demo_speed_current(ctx: BxiDeviceContext, uses_revo2_api: bool) -> None:
    print("\n=== Demo 2: Speed & Current Control ===")
    delay = 1.0

    print("[Demo] Speed control - single finger (middle)...")
    await ctx.handle.set_finger_speed(ctx.slave_id, libstark.FingerId.Middle, 500)
    await asyncio.sleep(delay)
    await ctx.handle.set_finger_speed(ctx.slave_id, libstark.FingerId.Middle, -500)
    await asyncio.sleep(delay)
    await ctx.handle.set_finger_speed(ctx.slave_id, libstark.FingerId.Middle, 0)

    print("[Demo] Speed control - all fingers...")
    await ctx.handle.set_finger_speeds(ctx.slave_id, [100, 100, 500, 500, 500, 500])
    await asyncio.sleep(delay)
    await ctx.handle.set_finger_speeds(ctx.slave_id, [-100, -100, -500, -500, -500, -500])
    await asyncio.sleep(delay)
    await ctx.handle.set_finger_speeds(ctx.slave_id, [0, 0, 0, 0, 0, 0])

    print("[Demo] Current control - single finger...")
    await ctx.handle.set_finger_current(ctx.slave_id, libstark.FingerId.Middle, 300)
    await asyncio.sleep(delay)
    await ctx.handle.set_finger_current(ctx.slave_id, libstark.FingerId.Middle, -300)
    await asyncio.sleep(delay)

    print("[Demo] Current control - all fingers...")
    await ctx.handle.set_finger_currents(ctx.slave_id, [200, 200, 300, 300, 300, 300])
    await asyncio.sleep(delay)
    await ctx.handle.set_finger_currents(ctx.slave_id, [-200, -200, -300, -300, -300, -300])
    await asyncio.sleep(delay)

    if uses_revo2_api:
        print("[Demo] PWM control (Revo2 only)...")
        await ctx.handle.set_finger_pwm(ctx.slave_id, libstark.FingerId.Middle, 700)
        await asyncio.sleep(delay)
        await ctx.handle.set_finger_pwm(ctx.slave_id, libstark.FingerId.Middle, -700)
        await asyncio.sleep(delay)

        await ctx.handle.set_finger_pwms(ctx.slave_id, [100, 100, 700, 700, 700, 700])
        await asyncio.sleep(delay)

    await ctx.handle.set_finger_positions(ctx.slave_id, [0, 0, 0, 0, 0, 0])
    await asyncio.sleep(delay)


async def demo_advanced_revo2(ctx: BxiDeviceContext) -> None:
    print("\n=== Demo 3: Advanced Control (Revo2 Only) ===")
    delay = 1.5

    print("[Demo] Setting unit mode to Normalized...")
    await ctx.handle.set_finger_unit_mode(ctx.slave_id, libstark.FingerUnitMode.Normalized)
    mode = await ctx.handle.get_finger_unit_mode(ctx.slave_id)
    print(f"  Current mode: {mode}")

    print("[Demo] Position + time control (single finger)...")
    await ctx.handle.set_finger_position_with_millis(
        ctx.slave_id, libstark.FingerId.Middle, 1000, 1000
    )
    await asyncio.sleep(delay)
    await ctx.handle.set_finger_position_with_millis(
        ctx.slave_id, libstark.FingerId.Middle, 0, 1000
    )
    await asyncio.sleep(delay)

    print("[Demo] Position + speed control (single finger)...")
    await ctx.handle.set_finger_position_with_speed(
        ctx.slave_id, libstark.FingerId.Middle, 1000, 50
    )
    await asyncio.sleep(delay)
    await ctx.handle.set_finger_position_with_speed(
        ctx.slave_id, libstark.FingerId.Middle, 0, 50
    )
    await asyncio.sleep(delay)

    print("[Demo] Position + duration control (all fingers)...")
    await ctx.handle.set_finger_positions_and_durations(
        ctx.slave_id,
        [300, 300, 500, 500, 500, 500],
        [500, 500, 500, 500, 500, 500],
    )
    await asyncio.sleep(delay)

    print("[Demo] Position + speed control (all fingers)...")
    await ctx.handle.set_finger_positions_and_speeds(
        ctx.slave_id,
        [100, 100, 800, 800, 800, 800],
        [300, 300, 300, 300, 300, 300],
    )
    await asyncio.sleep(delay)

    print("[Demo] Reading finger parameters...")
    max_pos = await ctx.handle.get_finger_max_position(ctx.slave_id, libstark.FingerId.Middle)
    min_pos = await ctx.handle.get_finger_min_position(ctx.slave_id, libstark.FingerId.Middle)
    max_speed = await ctx.handle.get_finger_max_speed(ctx.slave_id, libstark.FingerId.Middle)
    max_current = await ctx.handle.get_finger_max_current(ctx.slave_id, libstark.FingerId.Middle)
    protected_current = await ctx.handle.get_finger_protected_current(
        ctx.slave_id, libstark.FingerId.Middle
    )
    print("  Middle finger params:")
    print(f"    Max position: {max_pos}")
    print(f"    Min position: {min_pos}")
    print(f"    Max speed: {max_speed}")
    print(f"    Max current: {max_current}")
    print(f"    Protected current: {protected_current}")

    await ctx.handle.set_finger_positions(ctx.slave_id, [0, 0, 0, 0, 0, 0])
    await asyncio.sleep(delay)


async def demo_action_sequences(ctx: BxiDeviceContext) -> None:
    print("\n=== Demo 4: Action Sequences ===")
    delay = 1.5

    print("[Demo] Running built-in gestures...")

    print("  Open hand...")
    await ctx.handle.run_action_sequence(ctx.slave_id, libstark.ActionSequenceId.DefaultGestureOpen)
    await asyncio.sleep(delay)

    print("  Fist...")
    await ctx.handle.run_action_sequence(ctx.slave_id, libstark.ActionSequenceId.DefaultGestureFist)
    await asyncio.sleep(delay)

    print("  Two-finger pinch...")
    await ctx.handle.run_action_sequence(ctx.slave_id, libstark.ActionSequenceId.DefaultGesturePinchTwo)
    await asyncio.sleep(delay)

    print("  Three-finger pinch...")
    await ctx.handle.run_action_sequence(ctx.slave_id, libstark.ActionSequenceId.DefaultGesturePinchThree)
    await asyncio.sleep(delay)

    print("  Side pinch...")
    await ctx.handle.run_action_sequence(ctx.slave_id, libstark.ActionSequenceId.DefaultGesturePinchSide)
    await asyncio.sleep(delay)

    print("  Point...")
    await ctx.handle.run_action_sequence(ctx.slave_id, libstark.ActionSequenceId.DefaultGesturePoint)
    await asyncio.sleep(delay)

    print("  Open hand (reset)...")
    await ctx.handle.run_action_sequence(ctx.slave_id, libstark.ActionSequenceId.DefaultGestureOpen)
    await asyncio.sleep(delay)


async def demo_device_info(ctx: BxiDeviceContext, uses_revo2_api: bool) -> None:
    print("\n=== Demo 5: Device Info & Configuration ===")

    print("\n[Config] Communication:")
    rs485_baud = await ctx.handle.get_serialport_baudrate(ctx.slave_id)
    print(f"  RS485 Baudrate: {rs485_baud}")

    if uses_revo2_api:
        canfd_baud = await ctx.handle.get_canfd_baudrate(ctx.slave_id)
        print(f"  CANFD Baudrate: {canfd_baud}")

    print("\n[Config] System:")
    turbo = await ctx.handle.get_turbo_mode_enabled(ctx.slave_id)
    print(f"  Turbo mode: {'enabled' if turbo else 'disabled'}")

    auto_calibration = await ctx.handle.get_auto_calibration_enabled(ctx.slave_id)
    print(f"  Auto calibration: {'enabled' if auto_calibration else 'disabled'}")

    if uses_revo2_api:
        unit_mode = await ctx.handle.get_finger_unit_mode(ctx.slave_id)
        print(f"  Unit mode: {unit_mode}")

        print("\n[Config] Motor Parameters (Middle finger):")
        max_pos = await ctx.handle.get_finger_max_position(ctx.slave_id, libstark.FingerId.Middle)
        min_pos = await ctx.handle.get_finger_min_position(ctx.slave_id, libstark.FingerId.Middle)
        print(f"  Position range: {min_pos} - {max_pos}")

        max_speed = await ctx.handle.get_finger_max_speed(ctx.slave_id, libstark.FingerId.Middle)
        print(f"  Max speed: {max_speed}")

        max_current = await ctx.handle.get_finger_max_current(ctx.slave_id, libstark.FingerId.Middle)
        protected_current = await ctx.handle.get_finger_protected_current(
            ctx.slave_id, libstark.FingerId.Middle
        )
        print(f"  Max current: {max_current}, Protected: {protected_current}")


async def run_all_demos(ctx: BxiDeviceContext, title: str) -> None:
    print(f"\n================ {title} ================")
    await print_device_info(ctx)
    uses_revo2_api = True
    await demo_basic_position(ctx)
    await demo_speed_current(ctx, uses_revo2_api)
    await demo_advanced_revo2(ctx)
    await demo_action_sequences(ctx)
    await demo_device_info(ctx, uses_revo2_api)


async def async_main() -> int:
    left_ctx: Optional[BxiDeviceContext] = None
    right_ctx: Optional[BxiDeviceContext] = None

    try:
        libstark.init_logging()

        left_ctx = await init_bxipci_device(
            5,
            126,
            master_id=1,
            is_canfd=True,
            hw_type=libstark.StarkHardwareType.Revo2Basic,
        )
        right_ctx = await init_bxipci_device(
            6,
            127,
            master_id=1,
            is_canfd=True,
            hw_type=libstark.StarkHardwareType.Revo2Basic,
        )

        await asyncio.sleep(1.0)

        await run_all_demos(left_ctx, "Left hand")
        await run_all_demos(right_ctx, "Right hand")
        return 0
    finally:
        await cleanup_bxipci_device(right_ctx)
        await cleanup_bxipci_device(left_ctx)
        stop_bxipci_runtime()


def main() -> int:
    rclpy.init(args=None)
    try:
        return asyncio.run(async_main())
    except KeyboardInterrupt:
        return 0
    finally:
        stop_bxipci_runtime()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
