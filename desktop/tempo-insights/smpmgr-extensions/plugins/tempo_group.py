"""tempo_group.py - Tempo-BT Group 64 plugin for smpmgr

Place this file in a plugins directory and use:
    smpmgr --plugin-path=./plugins --ble Tempo-BT tempo session-list
    smpmgr --plugin-path=./plugins --ble Tempo-BT tempo session-list  yyymmdd/uniqueid
    smpmgr --plugin-path=./plugins --ble Tempo-BT tempo storage-info
    smpmgr --plugin-path=./plugins --ble Tempo-BT tempo led-on red
    smpmgr --plugin-path-./plugins --ble Tempo-BT tempo logger-start
"""

import asyncio
import logging
from enum import IntEnum, unique
from typing import cast, List, Dict, Any, Optional

import smp.error as smperr
import smp.message as smpmsg
import typer
from rich import print
from rich.console import Console
from rich.table import Table

from smpmgr.common import Options, connect_with_spinner, get_smpclient

app = typer.Typer(name="tempo", help="Tempo-BT custom commands (Group 64)")
logger = logging.getLogger(__name__)
console = Console()

# Constants matching mcumgr_custom.c
MGMT_GROUP_ID_TEMPO = 64

# Command IDs
TEMPO_MGMT_ID_SESSION_LIST = 0
TEMPO_MGMT_ID_SESSION_INFO = 1
TEMPO_MGMT_ID_STORAGE_INFO = 2
TEMPO_MGMT_ID_LED_CONTROL = 3
TEMPO_MGMT_ID_LOGGER_CONTROL = 4
TEMPO_MGMT_ID_SESSION_DELETE = 5
TEMPO_MGMT_ID_SETTINGS_GET = 6
TEMPO_MGMT_ID_SETTINGS_SET = 7


@unique
class TEMPO_RET_RC(IntEnum):
    """Tempo-specific return codes"""
    OK = 0
    ERROR = 1
    INVALID_STATE = 2
    INVALID_PARAM = 3


class TempoErrorV1(smperr.ErrorV1):
    _GROUP_ID = MGMT_GROUP_ID_TEMPO


class TempoErrorV2(smperr.ErrorV2[TEMPO_RET_RC]):
    _GROUP_ID = MGMT_GROUP_ID_TEMPO


# Session List Command
class SessionListRequest(smpmsg.ReadRequest):
    _GROUP_ID = MGMT_GROUP_ID_TEMPO
    _COMMAND_ID = TEMPO_MGMT_ID_SESSION_LIST


class SessionListResponse(smpmsg.ReadResponse):
    _GROUP_ID = MGMT_GROUP_ID_TEMPO
    _COMMAND_ID = TEMPO_MGMT_ID_SESSION_LIST
    
    sessions: List[Dict[str, Any]]
    count: int


class SessionList(SessionListRequest):
    _Response = SessionListResponse
    _ErrorV1 = TempoErrorV1
    _ErrorV2 = TempoErrorV2


# Storage Info Command
class StorageInfoRequest(smpmsg.ReadRequest):
    _GROUP_ID = MGMT_GROUP_ID_TEMPO
    _COMMAND_ID = TEMPO_MGMT_ID_STORAGE_INFO


class StorageInfoResponse(smpmsg.ReadResponse):
    _GROUP_ID = MGMT_GROUP_ID_TEMPO
    _COMMAND_ID = TEMPO_MGMT_ID_STORAGE_INFO
    
    backend: str
    free_bytes: int
    total_bytes: int
    used_percent: int


class StorageInfo(StorageInfoRequest):
    _Response = StorageInfoResponse
    _ErrorV1 = TempoErrorV1
    _ErrorV2 = TempoErrorV2


# LED Control Command
class LEDControlRequest(smpmsg.WriteRequest):
    _GROUP_ID = MGMT_GROUP_ID_TEMPO
    _COMMAND_ID = TEMPO_MGMT_ID_LED_CONTROL
    
    enable: bool
    r: int
    g: int
    b: int


class LEDControlResponse(smpmsg.WriteResponse):
    _GROUP_ID = MGMT_GROUP_ID_TEMPO
    _COMMAND_ID = TEMPO_MGMT_ID_LED_CONTROL
    
    enabled: bool
    r: int
    g: int
    b: int


class LEDControl(LEDControlRequest):
    _Response = LEDControlResponse
    _ErrorV1 = TempoErrorV1
    _ErrorV2 = TempoErrorV2


# Logger Control Command
class LoggerControlRequest(smpmsg.WriteRequest):
    _GROUP_ID = MGMT_GROUP_ID_TEMPO
    _COMMAND_ID = TEMPO_MGMT_ID_LOGGER_CONTROL
    
    action: str


class LoggerControlResponse(smpmsg.WriteResponse):
    _GROUP_ID = MGMT_GROUP_ID_TEMPO
    _COMMAND_ID = TEMPO_MGMT_ID_LOGGER_CONTROL
    
    state: str
    success: bool
    session_id: Optional[int] = None
    session_path: Optional[str] = None


class LoggerControl(LoggerControlRequest):
    _Response = LoggerControlResponse
    _ErrorV1 = TempoErrorV1
    _ErrorV2 = TempoErrorV2

class SettingsGetRequest(smpmsg.ReadRequest):
    _GROUP_ID = MGMT_GROUP_ID_TEMPO
    _COMMAND_ID = TEMPO_MGMT_ID_SETTINGS_GET


class SettingsGetResponse(smpmsg.ReadResponse):
    _GROUP_ID = MGMT_GROUP_ID_TEMPO
    _COMMAND_ID = TEMPO_MGMT_ID_SETTINGS_GET
    
    ble_name: str
    pps_enabled: bool
    pcb_variant: int
    log_backend: str


class SettingsGet(SettingsGetRequest):
    _Response = SettingsGetResponse
    _ErrorV1 = TempoErrorV1
    _ErrorV2 = TempoErrorV2


# Add Settings Set Command classes
class SettingsSetRequest(smpmsg.WriteRequest):
    _GROUP_ID = MGMT_GROUP_ID_TEMPO
    _COMMAND_ID = TEMPO_MGMT_ID_SETTINGS_SET
    
    # All fields are optional - only send what needs to be changed
    ble_name: Optional[str] = None
    pps_enabled: Optional[bool] = None
    pcb_variant: Optional[int] = None
    log_backend: Optional[str] = None


class SettingsSetResponse(smpmsg.WriteResponse):
    _GROUP_ID = MGMT_GROUP_ID_TEMPO
    _COMMAND_ID = TEMPO_MGMT_ID_SETTINGS_SET
    
    ble_name: str
    pps_enabled: bool
    pcb_variant: int
    log_backend: str
    success: bool
    note: Optional[str] = None  # For messages like "BLE name changes require reboot"


class SettingsSet(SettingsSetRequest):
    _Response = SettingsSetResponse
    _ErrorV1 = TempoErrorV1
    _ErrorV2 = TempoErrorV2


# Add Session Delete Command classes (missing from current plugin)
class SessionDeleteRequest(smpmsg.WriteRequest):
    _GROUP_ID = MGMT_GROUP_ID_TEMPO
    _COMMAND_ID = TEMPO_MGMT_ID_SESSION_DELETE
    
    session: str


class SessionDeleteResponse(smpmsg.WriteResponse):
    _GROUP_ID = MGMT_GROUP_ID_TEMPO
    _COMMAND_ID = TEMPO_MGMT_ID_SESSION_DELETE
    
    success: bool
    files_deleted: Optional[int] = None
    error: Optional[str] = None


class SessionDelete(SessionDeleteRequest):
    _Response = SessionDeleteResponse
    _ErrorV1 = TempoErrorV1
    _ErrorV2 = TempoErrorV2

# CLI Commands
@app.command(name="session-list")
def session_list(ctx: typer.Context) -> None:
    """List all logging sessions on the device."""
    options = cast(Options, ctx.obj)
    smpclient = get_smpclient(options)

    async def f() -> None:
        await connect_with_spinner(smpclient, options.timeout)
        
        response = await smpclient.request(SessionList())
        
        if response.sessions:
            table = Table(title=f"Tempo-BT Sessions ({response.count} found)")
            table.add_column("Type", style="cyan")
            table.add_column("Name", style="green")
            table.add_column("Size", style="yellow")
            
            for session in response.sessions:
                type_str = "DIR" if session.get("is_dir", False) else "FILE"
                name = session.get("name", "Unknown")
                size = session.get("size", 0)
                size_str = f"{size:,} bytes"
                table.add_row(type_str, name, size_str)
            
            console.print(table)
        else:
            console.print("No sessions found", style="yellow")

    asyncio.run(f())


@app.command(name="storage-info")
def storage_info(ctx: typer.Context) -> None:
    """Get storage statistics from the device."""
    options = cast(Options, ctx.obj)
    smpclient = get_smpclient(options)

    async def f() -> None:
        await connect_with_spinner(smpclient, options.timeout)
        
        response = await smpclient.request(StorageInfo())
        
        free_mb = response.free_bytes / (1024 * 1024)
        total_mb = response.total_bytes / (1024 * 1024)
        
        table = Table(title="Tempo-BT Storage Info")
        table.add_column("Property", style="cyan")
        table.add_column("Value", style="green")
        
        table.add_row("Backend", response.backend)
        table.add_row("Total Space", f"{total_mb:.1f} MB ({response.total_bytes:,} bytes)")
        table.add_row("Free Space", f"{free_mb:.1f} MB ({response.free_bytes:,} bytes)")
        table.add_row("Used", f"{response.used_percent}%")
        
        console.print(table)

    asyncio.run(f())


@app.command(name="led-on")
def led_on(
    ctx: typer.Context,
    color: str = typer.Argument(..., help="Color name (red,green,blue,etc) or hex #RRGGBB"),
) -> None:
    """Turn on the LED with specified color."""
    options = cast(Options, ctx.obj)
    smpclient = get_smpclient(options)
    
    # Color presets
    presets = {
        "red": (255, 0, 0),
        "green": (0, 255, 0),
        "blue": (0, 0, 255),
        "yellow": (255, 255, 0),
        "cyan": (0, 255, 255),
        "magenta": (255, 0, 255),
        "white": (255, 255, 255),
        "orange": (255, 128, 0),
    }
    
    # Parse color
    if color.startswith("#"):
        hex_color = color.lstrip("#")
        if len(hex_color) != 6:
            console.print("Error: Hex color must be #RRGGBB format", style="red")
            raise typer.Exit(1)
        try:
            r = int(hex_color[0:2], 16)
            g = int(hex_color[2:4], 16)
            b = int(hex_color[4:6], 16)
        except ValueError:
            console.print("Error: Invalid hex color", style="red")
            raise typer.Exit(1)
    elif color.lower() in presets:
        r, g, b = presets[color.lower()]
    else:
        console.print(f"Error: Unknown color '{color}'", style="red")
        console.print(f"Available colors: {', '.join(presets.keys())}")
        raise typer.Exit(1)

    async def f() -> None:
        await connect_with_spinner(smpclient, options.timeout)
        
        response = await smpclient.request(LEDControl(
            enable=True,
            r=r,
            g=g,
            b=b
        ))
        
        console.print(f"LED set to {color} (RGB {r},{g},{b})", style="green")

    asyncio.run(f())


@app.command(name="led-off")
def led_off(ctx: typer.Context) -> None:
    """Turn off LED override (return to app control)."""
    options = cast(Options, ctx.obj)
    smpclient = get_smpclient(options)

    async def f() -> None:
        await connect_with_spinner(smpclient, options.timeout)
        
        response = await smpclient.request(LEDControl(
            enable=False,
            r=0,
            g=0,
            b=0
        ))
        
        console.print("LED override disabled - returned to app control", style="green")

    asyncio.run(f())


@app.command(name="logger-start")
def logger_start(ctx: typer.Context) -> None:
    """Start logging (will auto-arm if needed)."""
    options = cast(Options, ctx.obj)
    smpclient = get_smpclient(options)

    async def f() -> None:
        await connect_with_spinner(smpclient, options.timeout)
        
        response = await smpclient.request(LoggerControl(action="start"))
        
        if response.success:
            console.print(f"Logger started successfully. State: {response.state}", style="green")
            
            if response.session_id is not None:
                console.print(f"Session ID: {response.session_id}")
                console.print(f"Session Path: {response.session_path}")
        else:
            console.print(f"Failed to start logger. State: {response.state}", style="red")

    asyncio.run(f())


@app.command(name="logger-stop")
def logger_stop(ctx: typer.Context) -> None:
    """Stop logging."""
    options = cast(Options, ctx.obj)
    smpclient = get_smpclient(options)

    async def f() -> None:
        await connect_with_spinner(smpclient, options.timeout)
        
        response = await smpclient.request(LoggerControl(action="stop"))
        
        if response.success:
            console.print(f"Logger stopped successfully. State: {response.state}", style="green")
        else:
            console.print(f"Failed to stop logger. State: {response.state}", style="red")

    asyncio.run(f())


@app.command(name="logger-arm")
def logger_arm(ctx: typer.Context) -> None:
    """Arm the logger."""
    options = cast(Options, ctx.obj)
    smpclient = get_smpclient(options)

    async def f() -> None:
        await connect_with_spinner(smpclient, options.timeout)
        
        response = await smpclient.request(LoggerControl(action="arm"))
        
        if response.success:
            console.print(f"Logger armed successfully. State: {response.state}", style="green")
        else:
            console.print(f"Failed to arm logger. State: {response.state}", style="red")

    asyncio.run(f())


@app.command(name="logger-disarm")
def logger_disarm(ctx: typer.Context) -> None:
    """Disarm the logger."""
    options = cast(Options, ctx.obj)
    smpclient = get_smpclient(options)

    async def f() -> None:
        await connect_with_spinner(smpclient, options.timeout)
        
        response = await smpclient.request(LoggerControl(action="disarm"))
        
        if response.success:
            console.print(f"Logger disarmed successfully. State: {response.state}", style="green")
        else:
            console.print(f"Failed to disarm logger. State: {response.state}", style="red")

    asyncio.run(f())


@app.command(name="logger-control")
def logger_control(
    ctx: typer.Context,
    action: str = typer.Argument(..., help="Action: start, stop, arm, or disarm"),
) -> None:
    """Generic logger control command."""
    valid_actions = ["start", "stop", "arm", "disarm"]
    
    if action not in valid_actions:
        console.print(f"Error: Invalid action '{action}'", style="red")
        console.print(f"Valid actions: {', '.join(valid_actions)}")
        raise typer.Exit(1)
    
    options = cast(Options, ctx.obj)
    smpclient = get_smpclient(options)

    async def f() -> None:
        await connect_with_spinner(smpclient, options.timeout)
        
        response = await smpclient.request(LoggerControl(action=action))
        
        if response.success:
            console.print(f"Logger action '{action}' successful. State: {response.state}", style="green")
            
            if action == "start" and response.session_id is not None:
                console.print(f"Session ID: {response.session_id}")
                console.print(f"Session Path: {response.session_path}")
        else:
            console.print(f"Logger action '{action}' failed. State: {response.state}", style="red")

    asyncio.run(f())

@app.command(name="session-delete")
def session_delete(
    ctx: typer.Context,
    session: str = typer.Argument(..., help="Session name (e.g., '20250117/7BF3655C')"),
    yes: bool = typer.Option(False, "--yes", "-y", help="Skip confirmation prompt"),
) -> None:
    """Delete a logging session and all its files."""
    options = cast(Options, ctx.obj)
    smpclient = get_smpclient(options)
    
    # Confirm deletion unless --yes flag is provided
    if not yes:
        confirm = typer.confirm(f"Are you sure you want to delete session '{session}'?")
        if not confirm:
            console.print("Deletion cancelled", style="yellow")
            raise typer.Exit()

    async def f() -> None:
        await connect_with_spinner(smpclient, options.timeout)
        
        response = await smpclient.request(SessionDelete(session=session))
        
        if response.success:
            console.print(f"Session '{session}' deleted successfully", style="green")
            if response.files_deleted is not None:
                console.print(f"Files deleted: {response.files_deleted}")
        else:
            console.print(f"Failed to delete session '{session}'", style="red")
            if response.error:
                console.print(f"Error: {response.error}", style="red")

    asyncio.run(f())

# Add CLI commands for settings
@app.command(name="settings-get")
def settings_get(ctx: typer.Context) -> None:
    """Get all device settings from non-volatile memory."""
    options = cast(Options, ctx.obj)
    smpclient = get_smpclient(options)

    async def f() -> None:
        await connect_with_spinner(smpclient, options.timeout)
        
        response = await smpclient.request(SettingsGet())
        
        table = Table(title="Tempo-BT Device Settings")
        table.add_column("Setting", style="cyan")
        table.add_column("Value", style="green")
        table.add_column("Description", style="yellow")
        
        table.add_row("BLE Name", response.ble_name, "Bluetooth advertising name")
        table.add_row("PPS Enabled", str(response.pps_enabled), "GPS pulse-per-second input")
        table.add_row("PCB Variant", f"0x{response.pcb_variant:02X}", "Hardware variant identifier")
        table.add_row("Log Backend", response.log_backend, "Storage backend (sdcard/littlefs)")
        
        console.print(table)

    asyncio.run(f())

@app.command(name="settings-set")
def settings_set(
    ctx: typer.Context,
    ble_name: Optional[str] = typer.Option(None, "--ble-name", help="Set Bluetooth advertising name (max 31 chars)"),
    pps_enabled: Optional[bool] = typer.Option(None, "--pps-enabled/--no-pps-enabled", help="Enable/disable GPS PPS input"),
    pcb_variant: Optional[int] = typer.Option(None, "--pcb-variant", help="Set PCB hardware variant (0-255)"),
    log_backend: Optional[str] = typer.Option(None, "--log-backend", help="Set log storage backend (sdcard/internal)"),
) -> None:
    """Set one or more device settings in non-volatile memory."""
    options = cast(Options, ctx.obj)
    smpclient = get_smpclient(options)
    
    # Check if any settings were provided
    if all(opt is None for opt in [ble_name, pps_enabled, pcb_variant, log_backend]):
        console.print("Error: No settings provided. Use --help to see available options.", style="red")
        raise typer.Exit(1)
    
    # Validate inputs
    if ble_name is not None and len(ble_name) > 31:
        console.print(f"Error: BLE name too long (max 31 chars, got {len(ble_name)})", style="red")
        raise typer.Exit(1)
    
    if pcb_variant is not None and not 0 <= pcb_variant <= 255:
        console.print(f"Error: PCB variant must be 0-255 (got {pcb_variant})", style="red")
        raise typer.Exit(1)
    
    if log_backend is not None and log_backend not in ["fatfs", "littlefs"]:
        console.print(f"Error: Invalid log backend '{log_backend}' (must be 'fatfs' or 'littlefs')", style="red")
        raise typer.Exit(1)

    async def f() -> None:
        await connect_with_spinner(smpclient, options.timeout)
        
        # Build request with only the fields that were specified
        request = SettingsSet()
        settings_to_change = []
        
        if ble_name is not None:
            request.ble_name = ble_name
            settings_to_change.append(f"BLE Name = '{ble_name}'")
        
        if pps_enabled is not None:
            request.pps_enabled = pps_enabled
            settings_to_change.append(f"PPS Enabled = {pps_enabled}")
        
        if pcb_variant is not None:
            request.pcb_variant = pcb_variant
            settings_to_change.append(f"PCB Variant = 0x{pcb_variant:02X}")
        
        if log_backend is not None:
            request.log_backend = log_backend
            settings_to_change.append(f"Log Backend = '{log_backend}'")
        
        console.print("Setting:", style="cyan")
        for setting in settings_to_change:
            console.print(f"  • {setting}", style="yellow")
        
        response = await smpclient.request(request)
        
        if response.success:
            console.print("\nSettings updated successfully!", style="green")
            
            # Show current values
            table = Table(title="Current Settings")
            table.add_column("Setting", style="cyan")
            table.add_column("Value", style="green")
            
            table.add_row("BLE Name", response.ble_name)
            table.add_row("PPS Enabled", str(response.pps_enabled))
            table.add_row("PCB Variant", f"0x{response.pcb_variant:02X}")
            table.add_row("Log Backend", response.log_backend)
            
            console.print(table)
            
            if response.note:
                console.print(f"\nNote: {response.note}", style="yellow")
        else:
            console.print("Failed to update settings", style="red")

    asyncio.run(f())

# Plugin export - this is what smpmgr looks for
plugin = app