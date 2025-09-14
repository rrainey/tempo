"""tempo_group.py - Tempo-BT Group 64 plugin for smpmgr

Place this file in a plugins directory and use:
    smpmgr --plugin-path ./plugins ble --name Tempo-BT tempo session-list
    smpmgr --plugin-path ./plugins ble --name Tempo-BT tempo storage-info
    smpmgr --plugin-path ./plugins ble --name Tempo-BT tempo led-on red
    smpmgr --plugin-path ./plugins ble --name Tempo-BT tempo logger-start
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


# Plugin export - this is what smpmgr looks for
plugin = app