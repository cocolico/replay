"""
wuji_sdk - Python bindings for Wuji SDK

Communicate with Wuji devices
"""

# This stub uses PEP 585 generics (list[X] / dict[K, V]) and ships with the package for
# Python 3.8 and up. Stubs are not executed, so the syntax is not a runtime constraint;
# the future import and the Optional[...] / Union[...] spelling below are kept for
# consistency with the rest of the codebase, which must stay runnable on 3.8.
from __future__ import annotations

from enum import Enum
from typing import Any, Optional, List, Tuple, Generic, TypeVar, Literal, overload, Union, Callable, AsyncIterator, TypedDict
from typing_extensions import TypeAlias, deprecated

import numpy as np
import numpy.typing as npt

__version__: str

# Full source commit for offline build identification, or "unknown" when unavailable.
__commit__: str

T = TypeVar("T")
# Deprecated: the per-profile calibration output dimension was removed.
# Calibration now always produces one hand model per side using the canonical
# baseline; these types remain only so existing calibrate(hand_profile=...)
# calls keep type-checking. Passing a value emits a DeprecationWarning.
class WujiHandProfile(str, Enum):
    WUJI_HAND = "wujihand"
    WUJI_HAND_2 = "wujihand2"

HandProfileKey = Literal["wujihand", "wujihand2"]
HandProfile = Union[WujiHandProfile, HandProfileKey]

class SdkUserInfo(TypedDict):
    user_id: str
    display_name: str
    description: Optional[str]
    external_id: Optional[str]
    is_default: bool
    created_at: str
    updated_at: str

class CalibrationMetric(TypedDict, total=False):
    label: str
    finger: Optional[str]
    finger_b: Optional[str]
    value: float
    min: float
    max: float
    error: float
    unit: str
    hint: str

class CalibrationFeedback(TypedDict, total=False):
    operation: str
    step_index: int
    step_total: int
    step_name: str
    state: str
    progress: float
    hold_elapsed: float
    hold_target: float
    collect_elapsed: float
    collect_target: float
    frames_collected: int
    variance_ok: bool
    constraints_ok: bool
    variance: float
    variance_target: float
    metrics: list[CalibrationMetric]
    hints: list[str]

class GloveCalibrationResult(TypedDict, total=False):
    poses_collected: int
    frames_per_pose: dict[str, int]
    handedness: str
    calibrated_urdf: str
    sdk_user: SdkUserInfo

class ErrorInfo(TypedDict):
    """Shape of the ``dict`` returned by ``describe_error`` and ``WujiHand2.describe_error``."""
    code: int
    name: str
    severity: str
    clear_policy: str
    is_test: bool
    desc: str
    cause: str
    resolution: str
    desc_zh: str
    """Chinese text with the same meaning as ``desc``; empty when the catalog has none."""
    cause_zh: str
    resolution_zh: str

class WujiException(Exception):
    """Public base class for all SDK-originated Python API failures.

    Every SDK failure raised by a public binding method is an instance of
    ``WujiException`` or one of its subclasses. Catch this class to handle
    SDK failures uniformly. Python protocol exceptions such as
    ``StopAsyncIteration`` and interpreter-level argument extraction errors
    are not SDK-originated failures and remain native Python exceptions.

    Common error types (identifiable by exception message):
        - DeviceNotFound: Device not found
        - DeviceMismatch: Device type mismatch
        - AmbiguousPath: Ambiguous path in multi-device scenario
        - InvalidPathFormat: Invalid path format
        - PathNotFound: Resource path does not exist
        - ParamNotSet: SDK parameter is defined but has no stored value yet
        - SchemaMismatch: Data type mismatch
        - Disconnected: Device disconnected
        - ConnectionTimeout: Connection timed out
        - OperationTimeout: Operation timed out
        - SerializeError: Serialization failed
        - DeserializeError: Deserialization failed
        - NoData: No data available
        - StreamClosed: Stream closed
        - UnsupportedOperation: Unsupported operation
        - Internal: Internal error

    Example:
        >>> try:
        ...     value = device.get("invalid.path")
        ... except WujiException as e:
        ...     print(f"SDK error: {e}")
    """
    ...

class NodeOfflineError(WujiException):
    """The addressed node is not connected, so the request was never sent.

    Typically the fingertip tactile module of one finger. The device decides
    this before sending, so this is not a timeout and retrying will not help.
    Check that the module is attached and powered.

    Example:
        >>> try:
        ...     status = hand.tactile_status("ring")
        ... except NodeOfflineError:
        ...     print("no fingertip module on the ring finger")
    """
    ...

class TransportType:
    """Transport type enumeration"""
    Usb: "TransportType"
    Udp: "TransportType"

class DeviceType:
    """Device type, known at scan time.

    Returned by :attr:`DiscoveredDevice.device_type`. ``Unknown`` means the
    type was not available at scan time or is not recognized by this SDK build.
    """
    Unknown: "DeviceType"
    WujiGlove: "DeviceType"
    WujiHand2: "DeviceType"
    WujiHand: "DeviceType"

class Handedness:
    """Handedness (left / right) for bimanual devices.

    Used with :meth:`SdkManager.connect` (``handedness=...``) to discover and
    connect a device by side. Currently relies on the Wuji Hand 2 firmware
    convention that SN[3] is ``'J'`` (left) or ``'K'`` (right); other devices
    will be filtered out at discovery.
    """
    Left: "Handedness"
    Right: "Handedness"

class ConnectOptions:
    """Connection options

    Attributes:
        timeout_ms: Timeout in milliseconds
        retry_count: Number of retries
        enable_bridge: Allow multiple SDK instances to connect to the same device
            (default True)
        auto_time_sync_interval_ms: Background time-sync interval in milliseconds.
            ``None`` disables the background task (first sync inside ``connect()``
            still runs). Must be ``>= 100`` if set. Default ``30000``.
    """
    timeout_ms: int
    retry_count: int
    enable_bridge: bool
    auto_time_sync_interval_ms: Optional[int]

    def __init__(
        self,
        timeout_ms: int = 1000,
        retry_count: int = 3,
        enable_bridge: bool = True,
        auto_time_sync_interval_ms: Optional[int] = 30000,
    ) -> None: ...

class Resource:
    """Resource descriptor

    Attributes:
        path: Resource path
        can_get: Supports GET operation
        can_set: Supports SET operation
        can_sub: Supports subscription
        can_pub: Supports publishing
        can_exec: Supports action execution
    """
    path: str
    can_get: bool
    can_set: bool
    can_sub: bool
    can_pub: bool
    can_exec: bool

class DiscoveredDevice:
    """Discovered device information

    Attributes:
        sn: Serial number
        address: Full address
        transport_type: Transport type
        device_type: Device type (known at scan time; ``DeviceType.Unknown`` if
            not available)
    """
    sn: str
    address: str
    transport_type: TransportType
    device_type: DeviceType

class Topic:
    """Topic descriptor

    Attributes:
        path: Topic path
        can_sub: Supports subscription
        can_pub: Supports publishing
    """
    path: str
    can_sub: bool
    can_pub: bool

class Param:
    """Parameter descriptor

    Attributes:
        path: Parameter path (dot-separated)
        can_get: Supports GET operation
        can_set: Supports SET operation
    """
    path: str
    can_get: bool
    can_set: bool

class NodeInfo:
    """Node information

    Attributes:
        path: Node path
    """
    path: str

class DeviceInfo:
    """Device information

    Attributes:
        serial_number: Device serial number
        firmware_version: Firmware version string
    """
    serial_number: str
    firmware_version: str

class Quaternion:
    """Quaternion orientation data

    Attributes:
        x: X component
        y: Y component
        z: Z component
        w: W component (scalar)
    """
    x: float
    y: float
    z: float
    w: float

class Subscription(Generic[T]):
    """Subscription handle

    Used to receive data streams from the device.

    Attributes:
        dtype: Data type name
        stream_id: Stream identifier
    """
    dtype: str
    stream_id: int

    def recv(self) -> Optional[T]:
        """Receive data (synchronous, non-blocking)

        Returns immediately, returns None if no data available.

        Returns:
            Received data or None

        Raises:
            WujiException: Connection disconnected
        """
        ...

    async def recv_async(self) -> T:
        """Receive data (asynchronous)

        Waits for the next data to arrive. Supports Python async/await syntax.

        Returns:
            Received data, type determined by dtype at creation

        Raises:
            WujiException: Receive failed or connection disconnected
        """
        ...

    def close(self) -> None:
        """Close subscription and release device-side resources

        After closing, subsequent recv calls will return an error.
        If not called, resources are released when the object is garbage collected.
        """
        ...

    def set_rate(self, frequency_hz: int) -> int:
        """Set the device push rate (samples per second) for this stream.

        Only streams produced directly by the device support rate control. Derived
        streams follow their source automatically, while global topics such as
        ``tf_static`` are aggregated by the SDK. Calling this method on either
        raises WujiException. The device quantizes the request to a supported rate
        (an integer division of the native rate) and returns the rate it applies.
        Pass 0 to restore the device default. The rate is shared by all subscribers
        of the stream (last-writer-wins). It resets to the device default when the
        last subscription to the stream is closed or the device reconnects.

        Args:
            frequency_hz: Target rate in Hz (0-65535); 0 = device default full rate

        Returns:
            The actual rate applied by the device (quantized to a supported rate)

        Raises:
            WujiException: Downstream stream, global topic such as ``tf_static``, or the
                firmware does not support rate control
            OverflowError: frequency_hz outside the 0-65535 range
        """
        ...

class CallbackSubscription:
    """Callback subscription handle

    Created via subscribe_with_callback(), automatically receives data
    in background thread and triggers callbacks.

    Attributes:
        dtype: Data type name
        is_active: Whether callback is still running
        received_count: Number of data frames received
        error_count: Number of errors occurred
    """
    dtype: str
    is_active: bool
    received_count: int
    error_count: int

    def close(self) -> None:
        """Stop callback and close subscription

        This method will:
        1. Stop background receiving thread
        2. Wait for current callback to complete
        3. Close underlying subscription
        """
        ...

    def __enter__(self) -> "CallbackSubscription":
        """Context manager entry"""
        ...

    def __exit__(self, *args: Any) -> None:
        """Context manager exit, automatically calls close()"""
        ...

class Publisher:
    """Publisher handle

    Used to send data streams to the device.

    Attributes:
        dtype: Data type name
    """
    dtype: str

    def send(self, data: Any) -> None:
        """Send data (synchronous)

        Considered successful once transport sends the data, does not wait for response.
        Suitable for high-frequency data sending scenarios.

        Args:
            data: Data to send

        Raises:
            WujiException: Data type mismatch or send failed
        """
        ...

    async def send_async(self, data: Any) -> None:
        """Send data (asynchronous)

        Data is serialized before sending, large data is automatically chunked.

        Args:
            data: Data to send

        Raises:
            WujiException: Data type mismatch or send failed
        """
        ...

    def close(self) -> None:
        """Close publisher and release device-side resources

        After closing, subsequent send calls will fail.
        If not called, resources are released when the object is garbage collected.
        """
        ...

class TimeSyncResult:
    """Result returned by ``WujiGlove.sync_time()``."""

    @property
    def offset_us(self) -> int:
        """Clock offset in microseconds. ``device_time = uptime_us + offset_us``."""
        ...

    @property
    def round_trip_us(self) -> int:
        """Round-trip time in microseconds. Smaller is more precise."""
        ...

    @property
    def synced_at_us(self) -> int:
        """UTC microseconds at the moment of measurement (NTP T4)."""
        ...

class WujiDevice:
    """Device handle

    Lightweight handle, safe to clone. Provides resource read/write and subscription.

    Attributes:
        device_name: Device name
        serial_number: Device serial number
        is_connected: Whether connected
    """
    device_name: str
    serial_number: str
    is_connected: bool

    def resources(self) -> List[Resource]:
        """Get resource list

        Returns:
            List of resources supported by the device
        """
        ...

    def params(self) -> List[Param]:
        """Get Param resource list

        Returns:
            List of Param resources
        """
        ...

    def topics(self) -> List[Topic]:
        """Get device-level Topic resource list

        Returns:
            List of device-level Topic resources
        """
        ...

    def get(self, path: str) -> Any:
        """Get resource value (synchronous)

        Args:
            path: Resource path (without leading slash), e.g., "firmware_version"

        Returns:
            Resource value

        Raises:
            WujiException: Resource does not exist or get failed
        """
        ...

    def set(self, path: str, value: Any) -> None:
        """Set resource value (synchronous)

        Args:
            path: Resource path
            value: Value to set

        Raises:
            WujiException: Invalid path format or set failed
        """
        ...

    def execute(self, path: str) -> None:
        """Execute ACTION (synchronous)

        Args:
            path: ACTION resource path, e.g., "reboot"

        Raises:
            WujiException: Resource does not exist, does not support ACTION, or execution failed
        """
        ...

    def subscribe(self, path: str) -> Subscription[Any]:
        """Subscribe to resource stream

        Args:
            path: Resource path

        Returns:
            Subscription handle

        Raises:
            WujiException: Resource does not exist or subscription failed
        """
        ...

    def publish(self, path: str) -> Publisher:
        """Create publish stream

        Args:
            path: Resource path

        Returns:
            Publisher handle

        Raises:
            WujiException: Resource does not exist or creation failed
        """
        ...


    def disconnect(self) -> None:
        """Disconnect

        Raises:
            WujiException: Disconnect failed
        """
        ...

class UserDataComponent(TypedDict):
    """One component in a user-data export/import/preview report."""

    device_sn: Optional[str]
    name: str
    state: str  # "present" | "missing" | "skipped"
    reason: Optional[str]
    path: Optional[str]

class UserDataExportReport(TypedDict):
    out_path: str
    owner_user_id: str
    owner_display_name: str
    device_sns: List[str]
    components: List[UserDataComponent]

class UserDataImportReport(TypedDict):
    owner_user_id: str
    owner_display_name: str
    device_sns: List[str]
    imported_at: str
    components: List[UserDataComponent]

class UserDataPreview(TypedDict):
    owner_user_id: str
    owner_display_name: str
    device_sns: List[str]
    created_at: str
    components: List[UserDataComponent]

class TactileModelPaths(TypedDict):
    dir: str
    safetensors: str
    npz: str

class SdkManager:
    """SDK Manager

    Singleton pattern, obtain via `SdkManager.instance()`.
    Provides device discovery, connection management, and path routing.
    """

    @staticmethod
    def instance() -> "SdkManager":
        """Get singleton instance

        Returns:
            SDK manager instance
        """
        ...

    def scan(self) -> List[DiscoveredDevice]:
        """Scan all available devices (USB + UDP)

        Returns:
            List of discovered devices

        Raises:
            WujiException: Scan failed
        """
        ...

    def create_user(
        self,
        display_name: str,
        *,
        description: Optional[str] = None,
        external_id: Optional[str] = None,
    ) -> SdkUserInfo:
        """Create an SDK user.

        The default user has an empty string ID and keeps the legacy SDK paths.
        `description` and `external_id` are keyword-only to avoid confusing a
        local SDK user note with an application-defined stable user ID.
        """
        ...

    def list_users(self) -> list[SdkUserInfo]:
        """List SDK users."""
        ...

    def current_user(self) -> SdkUserInfo:
        """Get the current SDK user."""
        ...

    def switch_user(self, user_id: str) -> SdkUserInfo:
        """Switch current SDK user.

        Connected devices reload their SDK parameter store from the selected
        user's persisted files.
        """
        ...

    def switch_to_default_user(self) -> SdkUserInfo:
        """Switch to the default SDK user.

        The default user keeps the legacy SDK parameter and calibration paths.
        """
        ...

    def update_user(
        self,
        user_id: str,
        *,
        display_name: Optional[str] = None,
        description: Optional[str] = None,
        external_id: Optional[str] = None,
        clear_external_id: bool = False,
        clear_description: bool = False,
    ) -> SdkUserInfo:
        """Update an SDK user."""
        ...

    def delete_user(self, user_id: str) -> None:
        """Delete an SDK user.

        Deleting the current user switches back to the default user.
        """
        ...

    def tactile_dir(self, user_id: str, device_sn: str) -> str:
        """Resolve the SDK-managed tactile data directory for a user and device.

        The returned path is for SDK data-management operations. Its internal
        layout is not an application storage contract.
        """
        ...

    def tactile_model_paths(self, user_id: str, device_sn: str) -> TactileModelPaths:
        """Resolve SDK-managed contact-model artifact paths for a user and device.

        This is a user-data management resolver, not a tactile training API;
        callers must not derive or depend on a directory layout from its values.
        """
        ...

    def tactile_run_dir(self, user_id: str, device_sn: str) -> str:
        """Resolve the SDK-managed tactile-run root for a user and device.

        Run names and the directory layout remain SDK implementation details.
        """
        ...

    def export_user_data(self, user_id: str, out_path: str) -> UserDataExportReport:
        """Export all of an SDK user's data (hand models + tactile model and
        calibration runs) into a zip bundle at ``out_path``.

        Everything the user owns is exported; each component is reported as
        present / missing / skipped.
        """
        ...

    def preview_user_data(self, zip_path: str) -> UserDataPreview:
        """Inspect a user-data bundle without importing it.

        Runs the full validation (zip-slip, size limits, sha256, bag contract,
        owner id) and reports the components it contains.
        """
        ...

    def import_user_data(self, zip_path: str) -> UserDataImportReport:
        """Import a user-data bundle, restoring it to the bundle owner's SDK user.

        Every present component is written and overwrites any existing file for
        that user; the owner user is created if it does not exist. The current
        user is not switched. Fails without writing anything if validation fails.
        """
        ...

    def connect(
        self,
        *,
        sn: Optional[str] = None,
        address: Optional[str] = None,
        handedness: Optional[Handedness] = None,
        device_name: str,
        options: Optional[ConnectOptions] = None,
    ) -> Union[WujiDevice, "WujiGlove", "WujiHand", "WujiHand2"]:
        """Connect to device

        Connect via serial number, address, or handedness. Exactly one of
        ``sn`` / ``address`` / ``handedness`` must be provided.

        Args:
            sn: Device serial number
            address: Device address (IP:port, serial port path, or
                WujiHand USB address `usb:0483:2000:<serial>`)
            handedness: Side of a bimanual device (``Handedness.Left`` /
                ``Handedness.Right``). SDK scans the network and filters
                discovered devices by SN; raises if zero or multiple devices
                match.
            device_name: Local handle/alias for this connection (required).
                Used for later access and path routing; the device to connect
                to is chosen by ``sn`` / ``address`` / ``handedness``, not by
                this name.
            options: Connection options (optional)

        Returns:
            WujiDevice, WujiGlove, WujiHand, or WujiHand2 — typed by the
            connected device's type, not by ``device_name``
        """
        ...

    def auto_connect(self, device_name: str) -> Union[WujiDevice, "WujiGlove", "WujiHand", "WujiHand2"]:
        """Auto-connect to the single device discovered on the bus

        Connects to the one device currently discovered. ``device_name`` is a
        **local handle/alias only** — it does NOT select or match a device and
        does NOT constrain the device type. Any string is accepted as long as it
        is non-empty and contains no ``/`` or ``.`` (path separators); case,
        language and symbols are kept as-is.

        The returned semantic type is determined by the type of the device
        **actually connected**, not by the string passed in. Passing
        ``"wuji_hand_2"`` while a glove is on the bus still returns a WujiGlove.

        Args:
            device_name: Local handle/alias for this connection (required). Not
                a device selector; does not affect which device is connected.

        Returns:
            WujiDevice, WujiGlove, WujiHand, or WujiHand2 — typed by the
            connected device's type, not by ``device_name``

        Raises:
            WujiException: No device found, multiple devices found, or
                connection failed; or ``device_name`` is empty or contains
                ``/`` or ``.``
        """
        ...

    def get_device(self, device_name: str) -> Optional[WujiDevice]:
        """Get connected device handle

        Args:
            device_name: Device name

        Returns:
            Device handle, or None if not found
        """
        ...

    def get_connected_devices(self) -> List[Tuple[str, WujiDevice]]:
        """Get all connected devices

        Returns:
            List of (device_name, device_handle) tuples
        """
        ...

    def disconnect(self, device_name: str) -> None:
        """Disconnect specified device

        Args:
            device_name: Device name

        Raises:
            WujiException: Disconnect failed
        """
        ...

    def disconnect_all(self) -> None:
        """Disconnect all devices

        Raises:
            WujiException: Disconnect failed
        """
        ...

    def get(self, path: str) -> Any:
        """Get resource value (via path routing)

        Path format:
        - Root node resource: device_name.param (e.g., left_hand.device_name)
        - Child node resource: device_name/node.param (e.g., left_hand/sensor.gain)

        Separator rules:
        - Root node: Use `.` to connect device name and parameter
        - Child node: Use `/` to access nodes, `.` to separate node levels

        Args:
            path: Resource path

        Returns:
            Resource value

        Raises:
            WujiException: Invalid path format, device does not exist, or get failed
        """
        ...

    def set(self, path: str, value: Any) -> None:
        """Set resource value (via path routing)

        Args:
            path: Resource path
            value: Value to set

        Raises:
            WujiException: Invalid path format, device does not exist, or set failed
        """
        ...

    def execute(self, path: str) -> None:
        """Execute ACTION (via path routing)

        Args:
            path: ACTION resource path

        Raises:
            WujiException: Invalid path format, resource does not exist, or execution failed
        """
        ...

    def subscribe(self, path: str) -> Subscription:
        """Subscribe to resource stream (via path routing)

        Supports two path formats:
        - Device path: `device_name/node/topic`, e.g., "left_hand/sensor/tactile_matrix"
        - Global resource path (no device prefix): e.g., "tf_static", "tf" (cross-device merged subscription)

        Routing logic: first attempts device path resolution, falls back to global topic subscription.

        Args:
            path: Resource path

        Returns:
            Subscription handle

        Raises:
            WujiException: Path cannot be resolved as device resource or global resource
        """
        ...

    # --- GLOBAL RESOURCES BEGIN (auto-generated by wuji-idl) ---
    def tf_static(self) -> "TfStaticResource":
        """Static coordinate transforms (global topic)"""
        ...

    def tf(self) -> "TfResource":
        """Dynamic coordinate transforms (global topic)"""
        ...
    # --- GLOBAL RESOURCES END ---

    def publish(self, path: str) -> Publisher:
        """Create publish stream (via path routing)

        Path format: `device_name/node/topic`, e.g., "left_hand/test_pub"

        Args:
            path: Resource path (must start with device_name)

        Returns:
            Publisher handle

        Raises:
            WujiException: Invalid path format or creation failed
        """
        ...

    def get_all_topics(self) -> List[Tuple[str, Topic]]:
        """Get device-level topic list for all connected devices

        Returns device topics only; global topics (e.g. tf_static) are excluded.
        Use ``global_topics()`` to get global topics.

        Returns:
            List of (device_name, Topic) tuples
        """
        ...

    def global_topics(self) -> List[Topic]:
        """Get global topic list

        Global topics are not bound to a single device. The SDK internally
        aggregates data from all connected devices (e.g. tf_static).
        The returned list is deduplicated by path.

        Returns:
            List of global Topic resources
        """
        ...

def set_log_level(level: str) -> None:
    """Set SDK log level

    Controls the output level of SDK internal logs.
    Default level is INFO, can be set to DEBUG or TRACE for more details.

    Args:
        level: Log level, options: "trace", "debug", "info", "warn"/"warning", "error", "off"

    Raises:
        WujiException: Invalid log level
    """
    ...

def describe_error(full_code: str) -> Optional[ErrorInfo]:
    """Describe a device fault code as the SDK log prints it, e.g. ``"E-11-32-0102"``.

    Returns the ``ErrorInfo`` (name, severity, description, cause, suggested fix and
    ``clear_policy``: ``"AutoClear"``, ``"ManualClear"`` meaning it stays until cleared
    or the device restarts, ``"NonClearable"`` meaning it needs a restart), English by
    default. ``None`` when the code is unknown; then show the user the raw code and
    suggest updating the SDK. ``WujiHand2.describe_error`` stays for existing callers.
    """
    ...

# ===== Recording Engine Types =====

class ChannelStatus:
    """Channel data flow status"""
    is_online: bool

class ChannelHealth:
    """Per-channel health details

    Attributes:
        channel_name: Channel name
        channel_id: Channel ID
        actual_hz: Measured frequency (Hz)
        drop_rate: Frame drop rate (0.0-1.0)
        last_frame_age_ms: Time since last frame (ms), None if no frame received
        jitter_us: Inter-frame jitter in microseconds
        is_online: Whether the channel is currently receiving data
        last_downtime_ms: Duration of last downtime (ms), None if never dropped
    """
    channel_name: str
    channel_id: int
    actual_hz: float
    drop_rate: float
    last_frame_age_ms: Optional[int]
    jitter_us: float
    is_online: bool
    last_downtime_ms: Optional[int]

class QualityMetrics:
    """Real-time recording quality metrics

    Attributes:
        frame_drop_rate: Frame drop rate (0.0-1.0, 5s sliding window)
        frame_jitter_us: Inter-frame jitter in microseconds
        sync_offset_ms: Cross-channel sync offset in milliseconds
        sync_rate: Synchronization success rate (0.0-1.0)
        timestamp_ns: Nanosecond timestamp
        channels: Per-channel health details
    """
    frame_drop_rate: float
    frame_jitter_us: float
    sync_offset_ms: float
    sync_rate: float
    timestamp_ns: int
    channels: list[ChannelHealth]
    def __repr__(self) -> str: ...

class QualitySummary:
    """Recording quality summary

    Attributes:
        total_frames: Total frames received
        dropped_frames: Total frames dropped
        frame_drop_rate: Drop rate (0.0-1.0)
        avg_sync_offset_ms: Average sync offset in milliseconds
        max_sync_offset_ms: Maximum sync offset in milliseconds
        sync_rate: Synchronization success rate (0.0-1.0)
        spc_alert_count: Total SPC alerts triggered
        duration_s: Recording duration in seconds
    """
    total_frames: int
    dropped_frames: int
    frame_drop_rate: float
    avg_sync_offset_ms: float
    max_sync_offset_ms: float
    sync_rate: float
    spc_alert_count: int
    duration_s: float
    def __repr__(self) -> str: ...

class ChannelRecordingSummary:
    """Persisted messages for one MCAP channel, including zero-frame channels."""
    channel_id: int
    channel_name: str
    frame_count: int

class RecordingSummary:
    """Recording session summary returned by RecordingHandle.stop()

    Attributes:
        total_frames: Total frames written
        file_size: MCAP file size in bytes
        duration_s: Recording duration in seconds
        quality: Quality metrics summary
        channels: Persisted per-channel counts ordered by channel ID
    """
    total_frames: int
    file_size: int
    duration_s: float
    channels: List[ChannelRecordingSummary]
    quality: QualitySummary
    def __repr__(self) -> str: ...

class RecordingAlert:
    """Recording quality alert

    Attributes:
        metric: Name of the metric triggering alert
        current_value: Current measured value
        threshold: Alert threshold
        message: Human-readable alert message
    """
    metric: str
    current_value: float
    threshold: float
    message: str
    def __repr__(self) -> str: ...

class RecordingStatus:
    """Recording runtime status

    Attributes:
        state: Current state ("recording" or "paused")
        frame_count: Total frames written so far
        duration_s: Elapsed duration in seconds
    """
    state: str
    frame_count: int
    duration_s: float
    def __repr__(self) -> str: ...

class MetricsStream:
    """Async iterator yielding QualityMetrics updates

    Usage:
        async for metrics in handle.subscribe_metrics():
            print(metrics.frame_drop_rate)
    """
    def __aiter__(self) -> "MetricsStream": ...
    async def __anext__(self) -> QualityMetrics: ...
    def __repr__(self) -> str: ...

class StatusStream:
    """Async iterator yielding RecordingStatus updates

    Usage:
        async for status in handle.subscribe_status():
            print(status.state)
    """
    def __aiter__(self) -> "StatusStream": ...
    async def __anext__(self) -> RecordingStatus: ...
    def __repr__(self) -> str: ...

class AlertStream:
    """Async iterator yielding RecordingAlert notifications

    Usage:
        async for alert in handle.subscribe_alerts():
            print(alert.message)
    """
    def __aiter__(self) -> "AlertStream": ...
    async def __anext__(self) -> RecordingAlert: ...
    def __repr__(self) -> str: ...

class RecordingHandle:
    """Recording control handle

    Provides pause/resume/stop control and real-time quality monitoring.
    Obtained from TopicRecorder.start().
    """
    async def pause(self) -> None:
        """Pause the recording"""
        ...
    async def resume(self) -> None:
        """Resume a paused recording"""
        ...
    async def stop(self) -> RecordingSummary:
        """Stop recording and return summary

        Returns:
            RecordingSummary: Recording statistics and quality summary
        """
        ...
    def subscribe_metrics(self) -> MetricsStream:
        """Subscribe to real-time quality metrics stream

        Returns:
            MetricsStream: Async iterator yielding QualityMetrics
        """
        ...
    def subscribe_status(self) -> StatusStream:
        """Subscribe to recording status stream

        Returns:
            StatusStream: Async iterator yielding RecordingStatus
        """
        ...
    def subscribe_alerts(self) -> AlertStream:
        """Subscribe to recording alert stream

        Returns:
            AlertStream: Async iterator yielding RecordingAlert
        """
        ...
    def __repr__(self) -> str: ...

class TopicRecorder:
    """MCAP recording session configurator

    Register channels via record() and start recording with start().
    Supports episode quick-switch: reuse session config, change output path.

    Example:
        session = TopicRecorder(compression="lz4")
        session.record(glove.tactile().subscribe())
        handle = await session.start("./data/episode.mcap")
        summary = await handle.stop()
    """
    def __init__(self, compression: str = "lz4", chunk_size: Optional[int] = None) -> None:
        """Create a recording session

        Args:
            compression: Compression algorithm ("lz4", "zstd", or "none")
            chunk_size: MCAP chunk size in bytes (default: 4MB)
        """
        ...
    def record(self, sub: "Subscription") -> None:
        """Register a subscription channel for recording

        Uses the topic and device info from the Subscription metadata.

        Args:
            sub: Subscription object from device.xxx().subscribe() or manager.xxx().subscribe()
        """
        ...
    async def start(self, output_path: str) -> RecordingHandle:
        """Start recording to an MCAP file

        Creates the MCAP file and begins recording all registered channels.
        Can be called multiple times for episode quick-switch.

        Args:
            output_path: Output MCAP file path

        Returns:
            RecordingHandle: Recording control handle
        """
        ...
    def __repr__(self) -> str: ...




class FingertipInfoResource:
    """A finger's info accessor, from ``hand.fingertip_<finger>_info()``.

    The finger is bound in the accessor, so ``get()`` takes no argument. The
    chunked info read is driven inside the SDK; the chunked-transport structs
    are internal and never exposed.
    """
    def get(self) -> FingertipSensorInfo: ...

# ===== AUTO-GENERATED BY WUJI-IDL BELOW — DO NOT EDIT =====

class FrameHeader:
    """Common frame header for timestamped data"""
    seq: int
    timestamp_us: int
    frame_id: str
    def __repr__(self) -> str: ...

class Vector3:
    """3D vector with f32 components"""
    x: float
    y: float
    z: float
    def __repr__(self) -> str: ...

class Vector3F64:
    """3D vector with f64 components"""
    x: float
    y: float
    z: float
    def __repr__(self) -> str: ...

class Pose:
    """Position and orientation in 3D space"""
    position: list[float]
    orientation: Quaternion
    def __repr__(self) -> str: ...
    def __init__(self, position: list[float], orientation: Quaternion) -> None: ...

class HwVersion:
    """Hardware version (Major.Minor.Patch). All-zero = unprovisioned at factory."""
    major: int
    minor: int
    patch: int
    def __repr__(self) -> str: ...
    def __init__(self, major: int, minor: int, patch: int) -> None: ...

class FingertipSensorInfo:
    """Fingertip sensor metadata (one per link; format JSON describes data payload layout)"""
    header: FrameHeader
    digest: int
    model: str
    device_type: int
    rate_hz: float
    format: str
    def __repr__(self) -> str: ...

class FingertipSensorData:
    """Fingertip sensor data stream (pure value payload, interpreted per info.format)"""
    header: FrameHeader
    info_digest: int
    data: list[int]
    def __repr__(self) -> str: ...
    def __len__(self) -> int: ...

class FingertipPose:
    """Fingertip pose with confidence"""
    pose: Pose
    confidence: float
    def __repr__(self) -> str: ...

class FingertipPoses:
    """Array of fingertip poses"""
    header: FrameHeader
    poses: list[FingertipPose]
    def __repr__(self) -> str: ...
    def __len__(self) -> int: ...

class FingerJointAngles:
    """Joint angles for a single finger"""
    angles: list[float]
    confidence: float
    def __repr__(self) -> str: ...

class HandJointAngles:
    """Joint angles for all fingers"""
    header: FrameHeader
    fingers: list[FingerJointAngles]
    def __repr__(self) -> str: ...
    def __len__(self) -> int: ...

class SkeletonJoint:
    """A single skeleton joint with pose and confidence"""
    name: str
    pose: Pose
    confidence: float
    def __repr__(self) -> str: ...

class HandSkeleton:
    """Hand skeleton with MediaPipe landmark joints"""
    header: FrameHeader
    joints: list[SkeletonJoint]
    def __repr__(self) -> str: ...
    def __len__(self) -> int: ...

class ImuData:
    """IMU sensor data with orientation, angular velocity, and linear acceleration"""
    header: FrameHeader
    orientation: Quaternion
    orientation_covariance: list[float]
    angular_velocity: Vector3F64
    angular_velocity_covariance: list[float]
    linear_acceleration: Vector3F64
    linear_acceleration_covariance: list[float]
    def __repr__(self) -> str: ...

class MitParam:
    """MIT control parameters for a single joint (kp, kd)"""
    kp: float
    kd: float
    def __repr__(self) -> str: ...
    def __init__(self, kp: float, kd: float) -> None: ...

class MitParams:
    """Vectorized MIT params — variable-length list of MitParam"""
    joints: list[MitParam]
    def __repr__(self) -> str: ...
    def __len__(self) -> int: ...

class ErrorStatus:
    """Per-joint active error codes by severity — one device fault code per severity level (u16, 0 = no active error at that level). Every non-zero value is a device fault code, same values as error_code_current; pass it to describe_error for name/severity/cause/resolution."""
    warning: int
    deferred_stop: int
    immediate_stop: int
    fatal: int
    def __repr__(self) -> str: ...
    def __init__(self, warning: int, deferred_stop: int, immediate_stop: int, fatal: int) -> None: ...

class ErrorLogEntry:
    """One error history record. error_code is a device fault code — same values as error_code_current; pass it to describe_error for name/severity/cause/resolution. flags packs is_first_fault(bit0) + ts_unit(bit1, 0=boot/1=wall µs)"""
    error_code: int
    flags: int
    timestamp_us: int
    uptime_us: int
    def __repr__(self) -> str: ...
    def __init__(self, error_code: int, flags: int, timestamp_us: int, uptime_us: int) -> None: ...

class ErrorLog:
    """Per-joint error history (variable length)."""
    entries: list[ErrorLogEntry]
    def __repr__(self) -> str: ...
    def __len__(self) -> int: ...

class Enc1CalibResult:
    """Encoder1 (motor-side) calibration result. The two error fields are unrelated: enc1_error_code reports why this calibration step failed (a small calibration-only enum, NOT accepted by describe_error); system_error_code is the joint's device fault code, same values as error_code_current, and is the one to pass to describe_error."""
    success: int
    enc1_error_code: int
    verify_dc_deg: float
    verify_ac_deg: float
    electrical_zero_offset: int
    system_error_code: int
    def __repr__(self) -> str: ...

class Enc2CalibResult:
    """Encoder2 (output-shaft limit) calibration result. The two error fields are unrelated: enc2_error_code reports why this calibration step failed (a small calibration-only enum, NOT accepted by describe_error); system_error_code is the joint's device fault code, same values as error_code_current, and is the one to pass to describe_error."""
    enc2_status: int
    enc2_error_code: int
    enc2_range_deg: float
    system_error_code: int
    def __repr__(self) -> str: ...

class JointEntry:
    """Single joint state snapshot (one element of JointState)."""
    nid: int
    status: int
    target_pos: float
    actual_pos: float
    target_vel: float
    actual_vel: float
    torque: float
    bcast_seq: int
    def __repr__(self) -> str: ...

class JointState:
    """Aggregated joint state frame; variable-length up to 20 joints. `num_joints` is a redundant header field to match `joints_len` (sanity check / readability)."""
    cycle_id: int
    num_joints: int
    joints: list[JointEntry]
    def __repr__(self) -> str: ...
    def __len__(self) -> int: ...

class JointStateEntry:
    """Single joint state (position / velocity / effort). `nid` is the firmware stream node id (`bus * 5 + node`, node 1..=4 = joints, node 5 = tactile slot, so joint nids are 1-4, 6-9, 11-14, 16-19, 21-24); convert it to the flat 0..=19 joint index with nid_to_joint_index (Python `WujiHand2.nid_to_joint_index`, C `wuji_hand_2_nid_to_joint_index`)."""
    nid: int
    position: float
    velocity: float
    effort: float
    def __repr__(self) -> str: ...

class JointStateFrame:
    """Whole-hand joint state frame (position/velocity/effort per joint); variable-length, online joints only (look up by `nid`)."""
    header: FrameHeader
    num_joints: int
    joints: list[JointStateEntry]
    def __repr__(self) -> str: ...
    def __len__(self) -> int: ...

class JointDiagnosticsEntry:
    """Single joint diagnostics (status word / current / bus voltage / temperature / error code / bus comm quality). `nid` is the firmware stream node id (`bus * 5 + node`, node 1..=4 = joints, node 5 = tactile slot, so joint nids are 1-4, 6-9, 11-14, 16-19, 21-24); convert it to the flat 0..=19 joint index with nid_to_joint_index (Python `WujiHand2.nid_to_joint_index`, C `wuji_hand_2_nid_to_joint_index`). error_code_current is this joint's active device fault code (u16, 0 = no active error); pass it to describe_error for name/severity/cause/resolution."""
    nid: int
    status_word: StatusWord
    current: float
    vbus_v_fb: float
    mcu_temp_c_fb: float
    error_code_current: int
    comm_response_rate_pct: int
    comm_timeout_total: int
    def __repr__(self) -> str: ...

class Hand2CommSummary:
    """Frame-level communication summary — SDK-local E2E stream/RPC stats + firmware comm_diag snapshot digest."""
    age_ms: int
    tactile_online_mask: int
    e2e_received: int
    e2e_lost: int
    e2e_reordered: int
    e2e_duplicates: int
    e2e_window_loss_x100: int
    rpc_total: int
    rpc_retries: int
    rpc_timeouts: int
    comm_get_failures: int
    tactile_response_rate_pct: list[int]
    tactile_timeout_total: list[int]
    sdk_dropped: int
    def __repr__(self) -> str: ...

class JointDiagnosticsFrame:
    """Whole-hand joint diagnostics frame; variable-length, online joints only (look up by `nid`). Carries a frame-level communication summary."""
    header: FrameHeader
    num_joints: int
    joints: list[JointDiagnosticsEntry]
    comm: Hand2CommSummary
    def __repr__(self) -> str: ...
    def __len__(self) -> int: ...

class JointCommand:
    """Single joint command — position + velocity + effort."""
    position: float
    velocity: float
    effort: float
    def __repr__(self) -> str: ...
    def __init__(self, position: float, velocity: float, effort: float) -> None: ...

class JointCommandFrame:
    """Whole-hand joint command — seq + 20 per-joint commands."""
    seq: int
    joints: list[JointCommand]
    def __repr__(self) -> str: ...

class PointField:
    """Description of a single field in a point cloud"""
    name: str
    offset: int
    type: int
    def __repr__(self) -> str: ...

class PointCloud:
    """Point cloud data"""
    header: FrameHeader
    frame_id: str
    point_stride: int
    fields: list[PointField]
    data: list[int]
    def __repr__(self) -> str: ...
    def __len__(self) -> int: ...

class TactileFrame:
    """Processed tactile sensor frame"""
    header: FrameHeader
    data: list[float]
    def __repr__(self) -> str: ...
    def __len__(self) -> int: ...

class TactileRawFrame:
    """Raw tactile sensor frame"""
    header: FrameHeader
    data: list[int]
    def __repr__(self) -> str: ...
    def __len__(self) -> int: ...

class TactileZones:
    """Tactile data grouped by finger zones"""
    header: FrameHeader
    palm: list[float]
    thumb: list[float]
    index: list[float]
    middle: list[float]
    ring: list[float]
    pinky: list[float]
    def __repr__(self) -> str: ...

class TactileBinary:
    """NC baseline binary contact detection output (24x31 row-major; 1.0=contact, 0.0=no contact, -1.0=invalid/masked taxel)"""
    header: FrameHeader
    data: list[float]
    def __repr__(self) -> str: ...
    def __len__(self) -> int: ...

class TactileResidual:
    """NC baseline continuous contact signal (24x31 row-major signed residual actual-μ-bias; -1.0=invalid/masked taxel)"""
    header: FrameHeader
    data: list[float]
    def __repr__(self) -> str: ...
    def __len__(self) -> int: ...

class FrameTransform:
    """A single coordinate frame transform"""
    timestamp_us: int
    parent_frame_id: str
    child_frame_id: str
    translation: list[float]
    rotation: Quaternion
    def __repr__(self) -> str: ...

class EmfPose:
    """EMF-derived pose with confidence score"""
    pose: Pose
    confidence: float
    def __repr__(self) -> str: ...

class EmfPoseArray:
    """Array of EMF poses"""
    header: FrameHeader
    poses: list[EmfPose]
    def __repr__(self) -> str: ...
    def __len__(self) -> int: ...

class FrameTransforms:
    """Collection of coordinate frame transforms"""
    transforms: list[FrameTransform]
    def __repr__(self) -> str: ...
    def __len__(self) -> int: ...

class HandJointStates:
    """ROS sensor_msgs/JointState-style joint state; position always present, velocity / effort optional (length 0 = channel not provided). Joint order is finger-major ({left,right}_finger{1..5}_joint{1..4})."""
    header: FrameHeader
    position: list[float]
    velocity: list[float]
    effort: list[float]
    def __repr__(self) -> str: ...

class HandJointCommand:
    """Joint command message for the joint_command topic; position/velocity/effort are parallel lists indexed by joint"""
    seq: int
    position: list[float]
    velocity: list[float]
    effort: list[float]
    def __repr__(self) -> str: ...

class TactileGloveStatus:
    """Health/availability of the paired tactile glove sub-module"""
    state: int
    last_frame_age_ms: int
    def __repr__(self) -> str: ...

class TactileGloveFrame:
    """Single tactile pressure frame (24x32 f32, flat ~3 KB)"""
    handedness: int
    sequence: int
    timestamp_ms: int
    pressure: list[float]
    def __repr__(self) -> str: ...
    def __len__(self) -> int: ...

class TactileGloveDiagnostics:
    """Tactile glove diagnostics snapshot"""
    uptime_ms: int
    frame_count: int
    crc_err_count: int
    dropout_count: int
    usb_reset_count: int
    def __repr__(self) -> str: ...

class TactileGloveDeviceInfo:
    """Tactile glove identity (TBIM SN, hw/fw revs)"""
    serial: str
    hw_revision: list[int]
    fw_version: list[int]
    def __repr__(self) -> str: ...

class TransferStats:
    """Per transfer-type stats"""
    tx_total: int
    timeout_total: int
    tx_per_sec: int
    timeout_rate_pct: int
    def __repr__(self) -> str: ...

class NodeDiagnostics:
    """Per-node diagnostics"""
    node_type: int
    online: int
    ms_since_last_response: int
    request_total: int
    response_ok_total: int
    timeout_total: int
    response_rate_pct: int
    request_per_sec: int
    def __repr__(self) -> str: ...

class FingerCommunicationDiagnostics:
    """Per-finger L2/L3/per-node diagnostics"""
    tx_frame_total: int
    rx_frame_total: int
    tx_byte_total: int
    rx_byte_total: int
    crc_error_total: int
    frame_format_error_total: int
    uart_hw_error_total: int
    tx_frame_per_sec: int
    rx_frame_per_sec: int
    tx_kbps: int
    rx_kbps: int
    error_per_sec: int
    transfer_stats: list[TransferStats]
    nodes: list[NodeDiagnostics]
    def __repr__(self) -> str: ...

class HandCommunicationDiagnostics:
    """Whole-hand 1Hz aggregated diagnostics across 5 fingers"""
    fingers: list[FingerCommunicationDiagnostics]
    def __repr__(self) -> str: ...

class Enc1CalibResultVec:
    """Encoder 1 calibration results for all 20 joints."""
    joints: list[Enc1CalibResult]
    def __repr__(self) -> str: ...
    def __len__(self) -> int: ...

class Enc2CalibResultVec:
    """Encoder 2 calibration results for all 20 joints."""
    joints: list[Enc2CalibResult]
    def __repr__(self) -> str: ...
    def __len__(self) -> int: ...

class EffortLimitVec:
    """Per-joint current limit (A) for all 20 joints."""
    joints: list[float]
    def __repr__(self) -> str: ...
    def __len__(self) -> int: ...

class ControlModeVec:
    """Per-joint control mode (0=off 1=mit 2=pos 3=vel 4=current) for all 20 joints."""
    joints: list[int]
    def __repr__(self) -> str: ...
    def __len__(self) -> int: ...

class TfStaticResource:
    """Static coordinate transforms (global topic)

    Cross-device merged resource. Subscribe to receive aggregated data
    from all connected devices.
    """
    def subscribe(self) -> Subscription: ...
    def subscribe_with_callback(
        self,
        callback: Callable[[FrameTransforms], None],
        on_error: Optional[Callable[[str], None]] = None,
    ) -> CallbackSubscription: ...

class TfResource:
    """Dynamic coordinate transforms (global topic)

    Cross-device merged resource. Subscribe to receive aggregated data
    from all connected devices.
    """
    def subscribe(self) -> Subscription: ...
    def subscribe_with_callback(
        self,
        callback: Callable[[FrameTransforms], None],
        on_error: Optional[Callable[[str], None]] = None,
    ) -> CallbackSubscription: ...

class SnResource:
    """Device serial number"""
    def get(self) -> str: ...

class VersionResource:
    """Firmware version"""
    def get(self) -> str: ...

class HwVersionResource:
    """Hardware version (Major.Minor.Patch). 0.0.0 = not provisioned at factory."""
    def get(self) -> HwVersion: ...

class IpResource:
    """IP address"""
    def get(self) -> str: ...
    def set(self, value: str) -> None: ...

class PortResource:
    """Data port"""
    def get(self) -> int: ...
    def set(self, value: int) -> None: ...

class HandSideResource:
    """Hand side ("left"/"right")"""
    def get(self) -> str: ...

class TactileResource:
    """Calibrated tactile data"""
    def subscribe(self) -> Subscription: ...
    def subscribe_with_callback(
        self,
        callback: Callable[[TactileFrame], None],
        on_error: Optional[Callable[[str], None]] = None,
    ) -> CallbackSubscription: ...

class TactileZonesResource:
    """Zone-grouped tactile data"""
    def subscribe(self) -> Subscription: ...
    def subscribe_with_callback(
        self,
        callback: Callable[[TactileZones], None],
        on_error: Optional[Callable[[str], None]] = None,
    ) -> CallbackSubscription: ...

class ImuPalmResource:
    """Palm IMU data"""
    def subscribe(self) -> Subscription: ...
    def subscribe_with_callback(
        self,
        callback: Callable[[ImuData], None],
        on_error: Optional[Callable[[str], None]] = None,
    ) -> CallbackSubscription: ...

class ImuThumbResource:
    """Thumb IMU data"""
    def subscribe(self) -> Subscription: ...
    def subscribe_with_callback(
        self,
        callback: Callable[[ImuData], None],
        on_error: Optional[Callable[[str], None]] = None,
    ) -> CallbackSubscription: ...

class ImuIndexResource:
    """Index finger IMU data"""
    def subscribe(self) -> Subscription: ...
    def subscribe_with_callback(
        self,
        callback: Callable[[ImuData], None],
        on_error: Optional[Callable[[str], None]] = None,
    ) -> CallbackSubscription: ...

class ImuMiddleResource:
    """Middle finger IMU data"""
    def subscribe(self) -> Subscription: ...
    def subscribe_with_callback(
        self,
        callback: Callable[[ImuData], None],
        on_error: Optional[Callable[[str], None]] = None,
    ) -> CallbackSubscription: ...

class ImuRingResource:
    """Ring finger IMU data"""
    def subscribe(self) -> Subscription: ...
    def subscribe_with_callback(
        self,
        callback: Callable[[ImuData], None],
        on_error: Optional[Callable[[str], None]] = None,
    ) -> CallbackSubscription: ...

class ImuPinkyResource:
    """Pinky finger IMU data"""
    def subscribe(self) -> Subscription: ...
    def subscribe_with_callback(
        self,
        callback: Callable[[ImuData], None],
        on_error: Optional[Callable[[str], None]] = None,
    ) -> CallbackSubscription: ...

class EmfPosesResource:
    """EMF pose estimation output"""
    def subscribe(self) -> Subscription: ...
    def subscribe_with_callback(
        self,
        callback: Callable[[EmfPoseArray], None],
        on_error: Optional[Callable[[str], None]] = None,
    ) -> CallbackSubscription: ...

class ImuDataPalmResource:
    """Palm fused IMU data"""
    def subscribe(self) -> Subscription: ...
    def subscribe_with_callback(
        self,
        callback: Callable[[ImuData], None],
        on_error: Optional[Callable[[str], None]] = None,
    ) -> CallbackSubscription: ...

class ImuDataThumbResource:
    """Thumb fused IMU data"""
    def subscribe(self) -> Subscription: ...
    def subscribe_with_callback(
        self,
        callback: Callable[[ImuData], None],
        on_error: Optional[Callable[[str], None]] = None,
    ) -> CallbackSubscription: ...

class ImuDataIndexResource:
    """Index finger fused IMU data"""
    def subscribe(self) -> Subscription: ...
    def subscribe_with_callback(
        self,
        callback: Callable[[ImuData], None],
        on_error: Optional[Callable[[str], None]] = None,
    ) -> CallbackSubscription: ...

class ImuDataMiddleResource:
    """Middle finger fused IMU data"""
    def subscribe(self) -> Subscription: ...
    def subscribe_with_callback(
        self,
        callback: Callable[[ImuData], None],
        on_error: Optional[Callable[[str], None]] = None,
    ) -> CallbackSubscription: ...

class ImuDataRingResource:
    """Ring finger fused IMU data"""
    def subscribe(self) -> Subscription: ...
    def subscribe_with_callback(
        self,
        callback: Callable[[ImuData], None],
        on_error: Optional[Callable[[str], None]] = None,
    ) -> CallbackSubscription: ...

class ImuDataPinkyResource:
    """Pinky finger fused IMU data"""
    def subscribe(self) -> Subscription: ...
    def subscribe_with_callback(
        self,
        callback: Callable[[ImuData], None],
        on_error: Optional[Callable[[str], None]] = None,
    ) -> CallbackSubscription: ...

class TipPosesResource:
    """Fingertip poses in wrist frame (from IK→FK)"""
    def subscribe(self) -> Subscription: ...
    def subscribe_with_callback(
        self,
        callback: Callable[[FingertipPoses], None],
        on_error: Optional[Callable[[str], None]] = None,
    ) -> CallbackSubscription: ...

class HandJointAnglesResource:
    """Hand joint angles from IK solver (21 DoF)"""
    def subscribe(self) -> Subscription: ...
    def subscribe_with_callback(
        self,
        callback: Callable[[HandJointAngles], None],
        on_error: Optional[Callable[[str], None]] = None,
    ) -> CallbackSubscription: ...

class HandSkeletonResource:
    """Hand skeleton (21 MediaPipe landmarks)"""
    def subscribe(self) -> Subscription: ...
    def subscribe_with_callback(
        self,
        callback: Callable[[HandSkeleton], None],
        on_error: Optional[Callable[[str], None]] = None,
    ) -> CallbackSubscription: ...

class TactilePointCloudResource:
    """3D tactile point cloud in wrist frame"""
    def subscribe(self) -> Subscription: ...
    def subscribe_with_callback(
        self,
        callback: Callable[[PointCloud], None],
        on_error: Optional[Callable[[str], None]] = None,
    ) -> CallbackSubscription: ...

class TactileResidualResource:
    """Continuous contact signal from NC baseline (24x31 row-major signed residual actual-μ-bias)"""
    def subscribe(self) -> Subscription: ...
    def subscribe_with_callback(
        self,
        callback: Callable[[TactileResidual], None],
        on_error: Optional[Callable[[str], None]] = None,
    ) -> CallbackSubscription: ...

class TactileBinaryResource:
    """Binary contact detection from NC baseline (24x31 row-major)"""
    def subscribe(self) -> Subscription: ...
    def subscribe_with_callback(
        self,
        callback: Callable[[TactileBinary], None],
        on_error: Optional[Callable[[str], None]] = None,
    ) -> CallbackSubscription: ...

class WujiGlove:
    """Intelligent data glove for robot teleoperation and human-computer interaction research, capturing real-time tactile and hand posture data."""

    serial_number: str
    device_name: str

    def __init__(self, device: WujiDevice) -> None: ...

    def sn(self) -> "SnResource":
        """Device serial number"""
        ...

    def version(self) -> "VersionResource":
        """Firmware version"""
        ...

    def hw_version(self) -> "HwVersionResource":
        """Hardware version (Major.Minor.Patch). 0.0.0 = not provisioned at factory."""
        ...

    def ip(self) -> "IpResource":
        """IP address"""
        ...

    def port(self) -> "PortResource":
        """Data port"""
        ...

    def hand_side(self) -> "HandSideResource":
        """Hand side ("left"/"right")"""
        ...

    def tactile(self) -> "TactileResource":
        """Calibrated tactile data"""
        ...

    def tactile_zones(self) -> "TactileZonesResource":
        """Zone-grouped tactile data"""
        ...

    def imu_palm(self) -> "ImuPalmResource":
        """Palm IMU data"""
        ...

    def imu_thumb(self) -> "ImuThumbResource":
        """Thumb IMU data"""
        ...

    def imu_index(self) -> "ImuIndexResource":
        """Index finger IMU data"""
        ...

    def imu_middle(self) -> "ImuMiddleResource":
        """Middle finger IMU data"""
        ...

    def imu_ring(self) -> "ImuRingResource":
        """Ring finger IMU data"""
        ...

    def imu_pinky(self) -> "ImuPinkyResource":
        """Pinky finger IMU data"""
        ...

    def reboot(self) -> None:
        """Reboot device"""
        ...

    def emf_poses(self) -> "EmfPosesResource":
        """EMF pose estimation output"""
        ...

    def imu_data_palm(self) -> "ImuDataPalmResource":
        """Palm fused IMU data"""
        ...

    def imu_data_thumb(self) -> "ImuDataThumbResource":
        """Thumb fused IMU data"""
        ...

    def imu_data_index(self) -> "ImuDataIndexResource":
        """Index finger fused IMU data"""
        ...

    def imu_data_middle(self) -> "ImuDataMiddleResource":
        """Middle finger fused IMU data"""
        ...

    def imu_data_ring(self) -> "ImuDataRingResource":
        """Ring finger fused IMU data"""
        ...

    def imu_data_pinky(self) -> "ImuDataPinkyResource":
        """Pinky finger fused IMU data"""
        ...

    def tip_poses(self) -> "TipPosesResource":
        """Fingertip poses in wrist frame (from IK→FK)"""
        ...

    def hand_joint_angles(self) -> "HandJointAnglesResource":
        """Hand joint angles from IK solver (21 DoF)"""
        ...

    def hand_skeleton(self) -> "HandSkeletonResource":
        """Hand skeleton (21 MediaPipe landmarks)"""
        ...

    def tactile_point_cloud(self) -> "TactilePointCloudResource":
        """3D tactile point cloud in wrist frame"""
        ...

    def tactile_residual(self) -> "TactileResidualResource":
        """Continuous contact signal from NC baseline (24x31 row-major signed residual actual-μ-bias)"""
        ...

    def tactile_binary(self) -> "TactileBinaryResource":
        """Binary contact detection from NC baseline (24x31 row-major)"""
        ...

    def sync_time(self) -> TimeSyncResult:
        """Trigger a single time sync round-trip and return the measured result.

        Returns ``TimeSyncResult(offset_us, round_trip_us, synced_at_us)``.
        Serialised with the background 30 s auto-sync task via an internal mutex.
        """
        ...
    def hand_model_path(self) -> "HandModelPathResource":
        """Custom hand URDF path accessor for online IK."""
        ...

    @staticmethod
    def offline_pipeline(
        sn: str,
        hand_side: Literal["left", "right"],
        urdf_path: Optional[str] = None,
    ) -> WujiGloveOfflinePipeline:
        """Create an offline processing pipeline.

        With the same SN, hand side, and URDF file, offline IK uses the same
        hand solver model as realtime IK. `urdf_path` pins a specific model for
        reproducible offline processing.
        """
        ...

    async def calibrate(
        self,
        *,
        skip_constraints: bool = False,
        timeout_s: float = 900.0,
        hand_profile: Optional[HandProfile] = None,
        on_feedback: Optional[Callable[[CalibrationFeedback], None]] = None,
    ) -> GloveCalibrationResult:
        """Run hand model calibration asynchronously.

        `hand_profile` is deprecated and ignored; calibration produces one hand
        model per side from the canonical baseline, and passing it emits a
        DeprecationWarning.

        Returns a summary dict containing `poses_collected`, `frames_per_pose`
        as a pose-name-to-frame-count map, `handedness`, `calibrated_urdf`, and
        `sdk_user`. `calibrated_urdf` is a local URDF file for offline replay or
        explicit application loading. Exceptions raised by `on_feedback` are
        logged and do not cancel the calibration.
        """
        ...

    def calibrate_blocking(
        self,
        *,
        skip_constraints: bool = False,
        timeout_s: float = 900.0,
        hand_profile: Optional[HandProfile] = None,
        on_feedback: Optional[Callable[[CalibrationFeedback], None]] = None,
    ) -> GloveCalibrationResult:
        """Run hand model calibration from synchronous Python code.

        `hand_profile` is deprecated and ignored; calibration produces one hand
        model per side from the canonical baseline, and passing it emits a
        DeprecationWarning.

        Returns a summary dict containing `poses_collected`, `frames_per_pose`
        as a pose-name-to-frame-count map, `handedness`, `calibrated_urdf`, and
        `sdk_user`. `calibrated_urdf` is a local URDF file for offline replay or
        explicit application loading. Exceptions raised by `on_feedback` are
        logged and do not cancel the calibration.

        Blocks the calling thread until calibration finishes or times out;
        Ctrl+C is deferred until the call returns. Use `calibrate` from
        asyncio code when responsive cancellation is needed.
        """
        ...

    async def calibrate_tactile(
        self,
        *,
        seconds_per_pose: float = 30.0,
        epochs: int = 60,
        install: bool = True,
        sensitivity: Optional[float] = None,
        timeout_s: float = 1800.0,
        on_feedback: Optional[Callable[[TactileCalibrationFeedback], None]] = None,
        on_pose_prompt: Optional[Callable[[TactilePosePrompt], Optional[Literal["proceed", "retry", "abort"]]]] = None,
    ) -> TactileCalibrationSummary:
        """Run the one-shot tactile calibration asynchronously.

        Guided collection, dataset validation, training, installation, and
        load validation. Feedback and the returned summary use the declared
        tactile calibration TypedDict schemas.

        `on_pose_prompt` makes it interactive: it's called before each motion
        with `{"kind": "pose_ready", ...}` and after each one with
        `{"kind": "pose_review", "warnings": [...], ...}` (`warnings` is empty
        when the pose looks fine). Return `"proceed"` / `None` to continue,
        `"retry"` to re-record, `"abort"` to stop. Omit it for a hands-off run.
        """
        ...

    def calibrate_tactile_blocking(
        self,
        *,
        seconds_per_pose: float = 30.0,
        epochs: int = 60,
        install: bool = True,
        sensitivity: Optional[float] = None,
        timeout_s: float = 1800.0,
        on_feedback: Optional[Callable[[TactileCalibrationFeedback], None]] = None,
        on_pose_prompt: Optional[Callable[[TactilePosePrompt], Optional[Literal["proceed", "retry", "abort"]]]] = None,
    ) -> TactileCalibrationSummary:
        """`calibrate_tactile` for synchronous scripts (blocks until done).

        See `calibrate_tactile` for `on_pose_prompt`.
        """
        ...

    # --- Base WujiDevice methods (delegated) ---

    info: Optional[DeviceInfo]
    is_connected: bool

    def resources(self) -> List[Resource]: ...
    def get(self, path: str) -> Any: ...
    def set(self, path: str, value: Any) -> None: ...
    def params(self) -> List[Param]: ...
    def topics(self) -> List[Topic]: ...
    def nodes(self) -> List[NodeInfo]: ...
    def subscribe(self, path: str) -> Subscription[Any]: ...
    def publish(self, path: str) -> Publisher: ...
    def execute(self, path: str) -> None: ...
    def save_params(self) -> None: ...
    def get_device_log_dir(self) -> str: ...
    def set_device_log_elf(self, elf_path: str) -> None: ...

    def disconnect(self) -> None: ...
    def __repr__(self) -> str: ...

class InputVoltageResource:
    """input_voltage"""
    def get(self) -> float: ...

class TemperatureResource:
    """temperature"""
    def get(self) -> float: ...

class HandednessResource:
    """handedness"""
    def get(self) -> int: ...

class FirmwareVersionResource:
    """firmware_version"""
    def get(self) -> int: ...

class ProductSnResource:
    """product_sn"""
    def get(self) -> str: ...

class WujiHandJointStatesResource:
    """joint_states"""
    def subscribe(self) -> Subscription: ...
    def subscribe_with_callback(
        self,
        callback: Callable[[HandJointStates], None],
        on_error: Optional[Callable[[str], None]] = None,
    ) -> CallbackSubscription: ...

class JointCommandResource:
    """joint_command"""
    def publish(self) -> JointCommandPublisher: ...

class TactileStatusResource:
    """tactile_status"""
    def subscribe(self) -> Subscription: ...
    def subscribe_with_callback(
        self,
        callback: Callable[[TactileGloveStatus], None],
        on_error: Optional[Callable[[str], None]] = None,
    ) -> CallbackSubscription: ...

class TactileDeviceInfoResource:
    """tactile_device_info"""
    def get(self) -> TactileGloveDeviceInfo: ...

class TactileHandednessResource:
    """tactile_handedness"""
    def get(self) -> int: ...

class TactilePressureFrameResource:
    """tactile_pressure_frame"""
    def subscribe(self) -> Subscription: ...
    def subscribe_with_callback(
        self,
        callback: Callable[[TactileGloveFrame], None],
        on_error: Optional[Callable[[str], None]] = None,
    ) -> CallbackSubscription: ...

class TactileDiagnosticsResource:
    """tactile_diagnostics"""
    def get(self) -> TactileGloveDiagnostics: ...

class WujiHand:
    """Canonical Wuji dexterous hand (5 fingers x 4 joints, USB CANopen) with optional tactile glove sub-module (USB CDC, 24x32 pressure)"""

    serial_number: str
    device_name: str

    def __init__(self, device: WujiDevice) -> None: ...

    def input_voltage(self) -> "InputVoltageResource":
        """input_voltage"""
        ...

    def temperature(self) -> "TemperatureResource":
        """temperature"""
        ...

    def handedness(self) -> "HandednessResource":
        """handedness"""
        ...

    def firmware_version(self) -> "FirmwareVersionResource":
        """firmware_version"""
        ...

    def product_sn(self) -> "ProductSnResource":
        """product_sn"""
        ...

    def joint_states(self) -> "WujiHandJointStatesResource":
        """joint_states"""
        ...

    def joint_command(self) -> "JointCommandResource":
        """joint_command"""
        ...

    def tactile_status(self) -> "TactileStatusResource":
        """tactile_status"""
        ...

    def tactile_device_info(self) -> "TactileDeviceInfoResource":
        """tactile_device_info"""
        ...

    def tactile_handedness(self) -> "TactileHandednessResource":
        """tactile_handedness"""
        ...

    def tactile_pressure_frame(self) -> "TactilePressureFrameResource":
        """tactile_pressure_frame"""
        ...

    def tactile_diagnostics(self) -> "TactileDiagnosticsResource":
        """tactile_diagnostics"""
        ...

    # --- Base WujiDevice methods (delegated) ---

    info: Optional[DeviceInfo]
    is_connected: bool

    def resources(self) -> List[Resource]: ...
    def get(self, path: str) -> Any: ...
    def set(self, path: str, value: Any) -> None: ...
    def params(self) -> List[Param]: ...
    def topics(self) -> List[Topic]: ...
    def nodes(self) -> List[NodeInfo]: ...
    def subscribe(self, path: str) -> Subscription[Any]: ...
    def publish(self, path: str) -> Publisher: ...
    def execute(self, path: str) -> None: ...
    def save_params(self) -> None: ...
    def get_device_log_dir(self) -> str: ...
    def set_device_log_elf(self, elf_path: str) -> None: ...

    def disconnect(self) -> None: ...
    def __repr__(self) -> str: ...

class CommDiagResource:
    """1Hz aggregated communication diagnostics across 5 fingers (L2/L3/per-node)"""
    def get(self) -> HandCommunicationDiagnostics: ...

class ImuResource:
    """IMU stream (accel/gyro as ImuData), ~100Hz."""
    def subscribe(self) -> Subscription: ...
    def subscribe_with_callback(
        self,
        callback: Callable[[ImuData], None],
        on_error: Optional[Callable[[str], None]] = None,
    ) -> CallbackSubscription: ...

class FingertipThumbDataResource:
    """Thumb fingertip sensor data stream (pure value payload; decode per info.format)"""
    def subscribe(self) -> Subscription: ...
    def subscribe_with_callback(
        self,
        callback: Callable[[FingertipSensorData], None],
        on_error: Optional[Callable[[str], None]] = None,
    ) -> CallbackSubscription: ...

class FingertipIndexDataResource:
    """Index fingertip sensor data stream (pure value payload; decode per info.format)"""
    def subscribe(self) -> Subscription: ...
    def subscribe_with_callback(
        self,
        callback: Callable[[FingertipSensorData], None],
        on_error: Optional[Callable[[str], None]] = None,
    ) -> CallbackSubscription: ...

class FingertipMiddleDataResource:
    """Middle fingertip sensor data stream (pure value payload; decode per info.format)"""
    def subscribe(self) -> Subscription: ...
    def subscribe_with_callback(
        self,
        callback: Callable[[FingertipSensorData], None],
        on_error: Optional[Callable[[str], None]] = None,
    ) -> CallbackSubscription: ...

class FingertipRingDataResource:
    """Ring fingertip sensor data stream (pure value payload; decode per info.format)"""
    def subscribe(self) -> Subscription: ...
    def subscribe_with_callback(
        self,
        callback: Callable[[FingertipSensorData], None],
        on_error: Optional[Callable[[str], None]] = None,
    ) -> CallbackSubscription: ...

class FingertipPinkyDataResource:
    """Pinky fingertip sensor data stream (pure value payload; decode per info.format)"""
    def subscribe(self) -> Subscription: ...
    def subscribe_with_callback(
        self,
        callback: Callable[[FingertipSensorData], None],
        on_error: Optional[Callable[[str], None]] = None,
    ) -> CallbackSubscription: ...

class JointStatesResource:
    """Whole-hand joint state (position / velocity / effort per joint)."""
    def subscribe(self) -> Subscription: ...
    def subscribe_with_callback(
        self,
        callback: Callable[[JointStateFrame], None],
        on_error: Optional[Callable[[str], None]] = None,
    ) -> CallbackSubscription: ...

class JointDiagnosticsResource:
    """Whole-hand joint diagnostics (status word / current / bus voltage / temperature / error code per joint)."""
    def subscribe(self) -> Subscription: ...
    def subscribe_with_callback(
        self,
        callback: Callable[[JointDiagnosticsFrame], None],
        on_error: Optional[Callable[[str], None]] = None,
    ) -> CallbackSubscription: ...

class WujiHand2:
    """Wuji Hand 2 dexterous hand (5 fingers x 4 joints)"""

    serial_number: str
    device_name: str

    def __init__(self, device: WujiDevice) -> None: ...

    def ip(self) -> "IpResource":
        """IP address"""
        ...

    def port(self) -> "PortResource":
        """Data port"""
        ...

    def hw_version(self) -> "HwVersionResource":
        """Hardware version (Major.Minor.Patch). 0.0.0 = not provisioned at factory."""
        ...

    def comm_diag(self) -> "CommDiagResource":
        """1Hz aggregated communication diagnostics across 5 fingers (L2/L3/per-node)"""
        ...

    def imu(self) -> "ImuResource":
        """IMU stream (accel/gyro as ImuData), ~100Hz."""
        ...

    def fingertip_thumb_data(self) -> "FingertipThumbDataResource":
        """Thumb fingertip sensor data stream (pure value payload; decode per info.format)"""
        ...

    def fingertip_index_data(self) -> "FingertipIndexDataResource":
        """Index fingertip sensor data stream (pure value payload; decode per info.format)"""
        ...

    def fingertip_middle_data(self) -> "FingertipMiddleDataResource":
        """Middle fingertip sensor data stream (pure value payload; decode per info.format)"""
        ...

    def fingertip_ring_data(self) -> "FingertipRingDataResource":
        """Ring fingertip sensor data stream (pure value payload; decode per info.format)"""
        ...

    def fingertip_pinky_data(self) -> "FingertipPinkyDataResource":
        """Pinky fingertip sensor data stream (pure value payload; decode per info.format)"""
        ...

    def joint_command(self) -> "JointCommandResource":
        """Whole-hand joint command stream — per-joint position / velocity / effort; ≤1kHz broadcast."""
        ...

    def reboot(self) -> None:
        """Reboot device"""
        ...

    def joint_states(self) -> "JointStatesResource":
        """Whole-hand joint state (position / velocity / effort per joint)."""
        ...

    def joint_diagnostics(self) -> "JointDiagnosticsResource":
        """Whole-hand joint diagnostics (status word / current / bus voltage / temperature / error code per joint)."""
        ...

    # --- Base WujiDevice methods (delegated) ---

    info: Optional[DeviceInfo]
    is_connected: bool

    def resources(self) -> List[Resource]: ...
    def get(self, path: str) -> Any: ...
    def set(self, path: str, value: Any) -> None: ...
    def params(self) -> List[Param]: ...
    def topics(self) -> List[Topic]: ...
    def nodes(self) -> List[NodeInfo]: ...
    def subscribe(self, path: str) -> Subscription[Any]: ...
    def publish(self, path: str) -> Publisher: ...
    def execute(self, path: str) -> None: ...
    def save_params(self) -> None: ...
    def get_device_log_dir(self) -> str: ...
    def set_device_log_elf(self, elf_path: str) -> None: ...

    def disconnect(self) -> None: ...
    def __repr__(self) -> str: ...


# ===== HAND-WRITTEN EXTENSIONS BELOW — preserved across regeneration =====

class HandModelPathResource:
    """Custom hand URDF path used by online IK (SDK param).

    Not supported for the default SDK user: its online IK always uses the
    built-in default URDF, so ``set()`` raises a ``WujiException`` explaining
    how to switch to a named SDK user first. ``set()`` also validates that the
    path points to a readable file and raises instead of storing an unusable
    path.
    """
    def get(self) -> str: ...
    def set(self, value: str) -> None: ...


class PointCloud(PointCloud):
    """Hand-written extension methods for PointCloud"""
    def point_count(self) -> int:
        """Get number of points in the cloud"""
        ...

class FrameHeader(FrameHeader):
    def __init__(self, seq: int, timestamp_us: int, frame_id: str = "") -> None: ...

class Quaternion(Quaternion):
    def __init__(self, x: float, y: float, z: float, w: float) -> None: ...

class Vector3F64(Vector3F64):
    def __init__(self, x: float, y: float, z: float) -> None: ...

class EmfPose(EmfPose):
    def __init__(self, pose: Pose, confidence: float) -> None: ...

class EmfPoseArray(EmfPoseArray):
    def __init__(self, header: FrameHeader, poses: list[EmfPose]) -> None: ...

class ImuData(ImuData):
    @staticmethod
    def raw(
        header: FrameHeader,
        angular_velocity: Vector3F64,
        linear_acceleration: Vector3F64,
    ) -> "ImuData": ...

class OfflineHandJointAnglesResource:
    def compute(self, input: EmfPoseArray) -> HandJointAngles: ...

class OfflineTipPosesResource:
    def compute(self, input: EmfPoseArray) -> FingertipPoses: ...

class OfflineHandSkeletonResource:
    def compute(self, input: EmfPoseArray) -> HandSkeleton: ...

class OfflineImuDataResource:
    def compute(self, input: ImuData) -> ImuData: ...

class WujiGloveOfflinePipeline:
    urdf_source: Literal["override", "calibration_file", "builtin_default"]
    urdf_source_path: Optional[str]
    sn: str
    hand_side: Literal["left", "right"]
    def hand_joint_angles(self) -> OfflineHandJointAnglesResource: ...
    def tip_poses(self) -> OfflineTipPosesResource: ...
    def hand_skeleton(self) -> OfflineHandSkeletonResource: ...
    def imu_data_palm(self) -> OfflineImuDataResource: ...
    def imu_data_thumb(self) -> OfflineImuDataResource: ...
    def imu_data_index(self) -> OfflineImuDataResource: ...
    def imu_data_middle(self) -> OfflineImuDataResource: ...
    def imu_data_ring(self) -> OfflineImuDataResource: ...
    def imu_data_pinky(self) -> OfflineImuDataResource: ...


# ──────────────────────────────────────────────────────────────────────────
# WujiGlove tactile calibration HAND-WRITTEN EXTENSIONS
# ──────────────────────────────────────────────────────────────────────────
# Stable v1 exposes only the guided one-shot calibration flow. The lower-level
# collection / dataset / training helpers remain Rust-internal until their
# contracts are ready to stabilize across Python and C.

class TactileCalibrationFeedback(TypedDict, total=False):
    operation: str
    state: str
    step_index: int
    step_total: int
    step_name: str
    collect_elapsed: float
    collect_target: float
    frames_collected: int
    epoch: int
    epoch_total: int
    best_val: float

class TactilePosePrompt(TypedDict, total=False):
    kind: str
    payload: dict
    step_index: int
    step_total: int
    step_name: str
    seconds_per_pose: float
    paired_frames: int
    warnings: List[str]

class TactileCalibrationSummary(TypedDict, total=False):
    poses_collected: int
    frames_per_pose: dict
    epochs: int
    best_val_loss: float
    alive_taxels: int
    dead_pct: float
    installed: bool
    model_dir: Optional[str]
    verified_alive_taxels: Optional[int]


# ──────────────────────────────────────────────────────────────────────────
# WujiHand HAND-WRITTEN EXTENSIONS
# ──────────────────────────────────────────────────────────────────────────
# Type stubs for the hand-written semantic APIs layered on top of the
# codegen-produced `WujiHand` class (Python conveniences + the
# RealtimeController context manager).

class WujiTactileGlove:
    """Tactile sub-module view — accessed via `WujiHand.tactile` property.

    Thin semantic wrappers over WujiHand's tactile GET helpers. Use
    `WujiHand.tactile_status().subscribe()` for the status topic.
    """

    def device_info(self) -> TactileGloveDeviceInfo:
        """Read tactile device identity and firmware metadata."""
        ...

    def handedness(self) -> int:
        """Read tactile handedness encoding (`0 = Left`, `1 = Right`)."""
        ...

    def handedness_name(self) -> Literal["Left", "Right", "Unknown"]:
        """Semantic decode of tactile handedness."""
        ...

    def diagnostics(self) -> TactileGloveDiagnostics:
        """Read tactile transport diagnostics."""
        ...

    def subscribe_pressure_frame(self) -> Subscription[TactileGloveFrame]:
        """Subscribe to `tactile/pressure_frame`."""
        ...

# ---- Wuji Hand 2 extension types ----

class CalibrationOutcome:
    """Calibration result for a single joint."""
    @property
    def status(self) -> str: ...
    @property
    def result(self) -> Optional["CalibrationResult"]: ...
    def __repr__(self) -> str: ...

class CalibrationReport:
    """Calibration report for all 20 joints."""
    def results(self) -> List[CalibrationOutcome]: ...
    def finger_results(self, name: str) -> List[CalibrationOutcome]: ...
    def __repr__(self) -> str: ...

class JointHandle:
    """Identifier for a single joint (label + flat index 0–19)."""
    @property
    def label(self) -> str: ...
    @property
    def index(self) -> int: ...
    def effort_limit(self) -> JointEffortLimitResource: ...
    def error_code(self) -> JointErrorCodeResource: ...
    def status_word(self) -> JointStatusWordResource: ...
    def enable(self) -> None: ...
    def disable(self) -> None: ...
    def clear_fault(self) -> None: ...
    def set_origin(self) -> None: ...
    def clear_origin(self) -> None: ...
    def constant(self) -> JointConstantResource: ...
    def position_table(self) -> JointPositionTableResource: ...
    def __repr__(self) -> str: ...

class FingerHandle:
    """Handle for one finger, providing access to its 4 joints."""
    def j0(self) -> JointHandle: ...
    def j1(self) -> JointHandle: ...
    def j2(self) -> JointHandle: ...
    def j3(self) -> JointHandle: ...
    def joints(self) -> List[JointHandle]: ...
    def __repr__(self) -> str: ...

class StatusWord:
    """Decoded status-word public view (low 16 bits): ext_state + limit flags."""
    ext_state: int
    position_limit_active: bool
    velocity_limit_active: bool
    current_limit_active: bool
    @property
    def ext_state_name(self) -> str: ...
    def __repr__(self) -> str: ...

class JointEffortLimitResource:
    """Single-joint effort limit (hand.joint(k).effort_limit()).

    The unit follows the control mode the joint is in: amperes (A) in MIT mode,
    joint output torque in newton-metres (N·m) in Position mode. Position mode
    and its N·m limit require firmware 3.0.0 or later; earlier firmware has a
    single limit in A.
    """
    def set(self, limit: float) -> None: ...
    def get(self) -> float: ...
    def __repr__(self) -> str: ...

class JointErrorCodeResource:
    """Single-joint current error code (hand.joint(k).error_code(), u16, includes warnings)."""
    def get(self) -> int: ...
    def __repr__(self) -> str: ...

class JointStatusWordResource:
    """Single-joint status word (hand.joint(k).status_word()); get() returns a decoded StatusWord."""
    def get(self) -> StatusWord: ...
    def __repr__(self) -> str: ...

class JointConstantResource:
    """Single-joint constant feed-forward mode (hand.joint(k).constant())."""
    def set(self, mode: Union[FeedForwardMode, int]) -> None:
        """Set the feed-forward mode using a ``FeedForwardMode`` or integer 0..=3."""
        ...
    def __repr__(self) -> str: ...

class JointPositionTableResource:
    """Single-joint position-indexed lookup-table feed-forward switch (hand.joint(k).position_table())."""
    def set(self, on: bool) -> None:
        """Enable/disable position-indexed lookup-table feed-forward. RAM only."""
        ...
    def __repr__(self) -> str: ...

class JointCommandPublisher:
    """Typed publisher for the `joint_command` PUB stream.

    Shared user-facing shape for first-generation WujiHand and Wuji Hand 2.
    Each backend serializes to its own command schema internally.

    On Wuji Hand 2 the same publisher and ``send`` serve both control modes:
    in MIT mode position, velocity and effort all take effect; in Position mode
    only position takes effect. A nonzero velocity or effort in Position mode is
    still sent, and the firmware silently ignores finite values. A successful send only
    confirms host publication, not device execution.

    Example:
        >>> pub = hand.joint_command().publish()
        >>> pub.send([JointCommand(0.0, 0.0, 0.0) for _ in range(20)])
        >>> pub.close()  # optional; publisher releases on drop
    """

    def send(self, joints: List[JointCommand]) -> None:
        """Send a command (AOS) for all 20 joints (list of 20 JointCommand)."""
        ...

    def close(self) -> None:
        """Close the publisher and release the PUB stream."""
        ...

class LowPass:
    """Low-pass filter configuration for the realtime controller.

    Currently only `cutoff_hz` is exposed; future filter
    families (Butterworth, etc.) will gain their own classes as upstream adds them.

    Example:
        >>> f = wuji_sdk.LowPass(cutoff_hz=5.0)
        >>> f.cutoff_hz
        5.0
    """

    cutoff_hz: float

    def __init__(self, cutoff_hz: float) -> None: ...
    def __repr__(self) -> str: ...

class RealtimeController:
    """Realtime controller handle, RAII'd via `with hand.realtime_controller(...) as ctrl:`.

    `__enter__` returns `self`; `__exit__` detaches only when no other SDK path
    still owns the shared controller.

    Thread-safe: one handle can be shared across Python threads (e.g. a writer
    streaming targets while a reader polls actual positions) without locking.

    Example:
        >>> with hand.realtime_controller(wuji_sdk.LowPass(cutoff_hz=5.0)) as ctrl:
        ...     while running:
        ...         ctrl.set_target_position([0.0] * 20)
        ...         pos = ctrl.get_actual_position()
    """

    def __enter__(self) -> "RealtimeController": ...
    def __exit__(self, exc_type, exc_val, exc_tb) -> bool: ...
    def set_target_position(self, position: List[float]) -> None:
        """Write filtered target positions (20 elements, radians). Raises WujiException on wrong length."""
        ...
    def get_actual_position(self) -> List[float]:
        """Read cached actual positions as a 20-element flat list (finger-major)."""
        ...
    def get_actual_effort(self) -> List[float]:
        """Read cached actual efforts (amps) as a 20-element flat list (finger-major)."""
        ...

class WujiHandJointHandle:
    """Single joint identifier (label + index) for WujiHand.

    Returned by `WujiHand.joints()`. Mirrors `WujiHand2.JointHandle` from the
    wuji_hand_2 binding so user code iterating joints can treat both device families
    uniformly. No SDO calls — purely an identifier.

    Example:
        >>> for j in hand.joints():
        ...     print(j.index, j.label)
        0 thumb_joint1
        1 thumb_joint2
        ...
        19 pinky_joint4
    """

    @property
    def index(self) -> int:
        """Flat joint index in finger-major order, 0..19."""
        ...

    @property
    def label(self) -> str:
        """Human-readable label `"<finger>_joint<j>"` (1-indexed `j`)."""
        ...

    def __repr__(self) -> str: ...


class WujiHandFingerHandle:
    """Finger identifier with accessors for its 4 joints.

    Returned by `WujiHand.fingers()`.

    Example:
        >>> for f in hand.fingers():
        ...     print(f.name, [j.label for j in f.joints()])
    """

    @property
    def name(self) -> str:
        """Finger name: "thumb" / "index" / "middle" / "ring" / "pinky"."""
        ...

    def joints(self) -> List[WujiHandJointHandle]:
        """All 4 joints of this finger."""
        ...

    def __repr__(self) -> str: ...


class WujiHandJointDiagnostics:
    """Per-joint diagnostics snapshot for WujiHand.

    Distinct from the wuji_hand_2 `JointDiagnostics` class (different field
    set).

    Returned (as a length-20 list) by `WujiHand.get_all_diagnostics()`.
    """

    @property
    def bus_voltage(self) -> float:
        """CAN bus voltage at this joint (volts)."""
        ...

    @property
    def temperature(self) -> float:
        """Joint controller temperature (°C)."""
        ...

    @property
    def error_code(self) -> int:
        """Firmware error code bitfield (one bit per fault condition); 0 = no fault.

        A first-generation-hand encoding of its own — unrelated to the fault
        codes in ``JointDiagnosticsEntry.error_code_current``, and not decodable by
        ``WujiHand2.describe_error``.
        """
        ...

    def __repr__(self) -> str: ...


class WujiHand(WujiHand):
    """Extension methods on the IDL-generated WujiHand class.

    Inheritance trick (`class WujiHand(WujiHand)`) layers hand-written stubs
    onto the auto-generated class without overwriting the auto-gen block.
    """

    @staticmethod
    def connect_sn(sn: str, device_name: str = "wuji_hand") -> "WujiHand":
        """Scan + connect by USB iSerial in one call — wujihandpy-style UX.

        Equivalent Python usage is
        `SdkManager.instance().connect(sn=sn, device_name=device_name)`.
        First-generation WujiHand devices are exposed as USB devices and routed
        to the native backend with tactile auto-pairing when available.
        """
        ...

    @property
    def device(self) -> WujiDevice:
        """The underlying `WujiDevice` for direct get/set/subscribe access."""
        ...

    def handedness_name(self) -> str:
        """Semantic decode of the hand handedness SDO.

        Hand-side encoding: `0 = Right`, `1 = Left` (device firmware convention).
        Returns `"Right"`, `"Left"`, or `"Unknown"`.
        """
        ...

    def is_tactile_attached(self) -> bool:
        """Whether a tactile glove is currently attached.

        Auto-paired at connect; flips to `False` if the glove is unplugged at
        runtime. Consumes one status frame from the `tactile_status()` stream.
        """
        ...

    @property
    def tactile(self) -> WujiTactileGlove:
        """Tactile sub-module accessor."""
        ...

    def realtime_controller(self, filter: LowPass) -> RealtimeController:
        """Open a realtime controller session, RAII'd through `with`.

        `LowPass` uses `cutoff_hz`; `enable_upstream` is internal and always
        enabled for WujiHand controller readback.

        See `RealtimeController` for usage example.

        Raises:
            WujiException: Device disconnected
        """
        ...

    # ── Wuji Hand 2-aligned batch methods (2026-05-26) ─────────────────────────

    def enable(self) -> None:
        """Enable all 20 joint motors.

        Mirrors `WujiHand2.enable()` from the wuji_hand_2 binding so user code
        can treat both device families uniformly.
        Raises `WujiException` if the device is disconnected.
        """
        ...

    def disable(self) -> None:
        """Disable all 20 joint motors.
        Raises `WujiException` if the device is disconnected.
        """
        ...

    def clear_all_faults(self) -> None:
        """Clear fault flags on all 20 joints (broadcast `ResetError=1`).
        Raises `WujiException` if the device is disconnected.
        """
        ...

    def set_all_effort_limit(self, amps: float) -> None:
        """Set effort limit (amps) for all 20 joints.
        Raises `WujiException` if the device is disconnected.
        """
        ...

    def get_all_effort_limit(self) -> List[float]:
        """Read effort limit (amps) for all 20 joints as a length-20 list.
        Raises `WujiException` if the device is disconnected.
        """
        ...

    def get_all_diagnostics(self) -> List[WujiHandJointDiagnostics]:
        """Read per-joint diagnostics (bus_voltage, temperature, error_code)
        for all 20 joints. Returns a length-20 list.
        Raises `WujiException` if the device is disconnected.
        """
        ...

    def get_soft_limits(self) -> tuple[List[float], List[float]]:
        """Read firmware-supplied soft limits (radians) for all 20 joints.

        Returns `(upper, lower)` — two length-20 lists. Read-only:
        the firmware does not expose a setter.
        Raises `WujiException` if the device is disconnected.
        """
        ...

    def read_joint_state(self) -> HandJointStates:
        """Foreground synchronous read of all 20 joint actual positions.

        Returns a `HandJointStates` (ROS sensor_msgs/JointState-style).
        Use this for one-shot snapshot queries; subscribe to
        `hand.joint_states()` for continuous streaming.

        Raises:
            WujiException: Device disconnected
        """
        ...

    def joints(self) -> List[WujiHandJointHandle]:
        """All 20 joints as a Python list of `WujiHandJointHandle` identifiers."""
        ...

    def fingers(self) -> List[WujiHandFingerHandle]:
        """All 5 fingers as a Python list of `WujiHandFingerHandle`."""
        ...

class HandednessResource:
    """Handedness resource (device-level, GET only). Returns 'left' or 'right'."""
    def get(self) -> str: ...
    def __repr__(self) -> str: ...

class DeviceEffortLimitResource:
    """Device-level effort limit resource.

    The unit follows the control mode the hand is in: amperes (A) in MIT mode,
    joint output torque in newton-metres (N·m) in Position mode. Each mode keeps
    its own limit, so call ``control_mode().set()`` before reading or writing it.
    Position-mode defaults differ per joint; to change them, scale the values
    ``get()`` returns. Position mode and its N·m limit require firmware 3.0.0
    or later; earlier firmware has a single limit in A.

    Polymorphic set: a scalar float applies to all
    20 joints; a length-20 list[float] (joint scan index 0..19) sets per-joint.
    get() returns list[20] of Optional[float] (offline joints are None).
    """
    @overload
    def set(self, value: float) -> None: ...
    @overload
    def set(self, value: List[float]) -> None: ...
    def get(self) -> List[Optional[float]]: ...
    def __repr__(self) -> str: ...

class DeviceMitParamsResource:
    """Device-level MIT params resource. Unit = one (kp, kd) bundle.

    Polymorphic set: a single
    (kp, kd) tuple or MitParam applies to all 20 joints; a length-20 list of
    (kp, kd) / MitParam (joint scan index 0..19) sets per-joint. get() returns a
    length-20 list[Optional[MitParam]]; raises on disconnect, offline joints are None.
    """
    @overload
    def set(self, value: Union[MitParam, Tuple[float, float]]) -> None: ...
    @overload
    def set(self, value: List[Union[MitParam, Tuple[float, float]]]) -> None: ...
    def get(self) -> List[Optional[MitParam]]: ...
    def __repr__(self) -> str: ...

class ControlModeResource:
    """Whole-hand control mode: one value for all 20 joints.

    ``set`` takes a ``ControlMode`` or its name (``"mit"`` / ``"position"``,
    case-insensitive). Lists and anything other than Mit or Position raise before
    a packet is sent.
    ``get`` returns one ``ControlMode``. When a joint is offline or the 20 joints
    disagree, it returns the last whole-hand mode read or written on this connection
    and logs a warning listing every joint; it raises only when there is none yet.
    Readback may also be Off, Velocity or Current.
    """
    def set(self, value: Union["ControlMode", str]) -> None: ...
    def get(self) -> "ControlMode": ...
    def __repr__(self) -> str: ...

class PositionCommandLowPassCutoffFrequencyResource:
    """Whole-hand position-command low-pass cutoff frequency in Hz.

    Takes effect in Position mode only. The firmware upsamples and smooths
    incoming targets at its control rate; a lower cutoff increases smoothing and
    delay. ``0 < hz < 1000`` filters at ``hz``; ``hz >= 1000`` turns the filter off;
    ``0`` selects the firmware default and reads back as 0. Negative and
    non-finite values raise before a packet is sent.
    ``set`` takes one value for all 20 joints and requires all joints online.
    ``get`` returns one value. When a joint is offline or the joints disagree, it
    returns the last whole-hand value read or written on this connection and logs a
    warning; it raises only when there is none yet. Only the cutoff changes; updates
    affect RAM only.
    """
    def set(self, hz: float) -> None: ...
    def get(self) -> float: ...
    def __repr__(self) -> str: ...

class ControlMode:
    """Joint control mode for Wuji Hand 2.

    Exposed so callers get autocompletion and edit-time typo detection instead
    of bare strings. Members compare equal to the firmware integer
    (``ControlMode.Mit == 1``). The legacy lowercase string spelling (e.g.
    ``"mit"``) is still accepted anywhere a ``ControlMode`` is expected.

    Only Mit and Position are accepted by ``control_mode().set()``. Off,
    Velocity and Current remain available for compatibility and readback.
    """
    Off: "ControlMode"
    Mit: "ControlMode"
    Position: "ControlMode"
    Velocity: "ControlMode"
    Current: "ControlMode"
    def __str__(self) -> str: ...
    def __repr__(self) -> str: ...

class FeedForwardMode:
    """Feed-forward terms selected by ``WujiHand2.set_constant``.

    Members compare equal to the firmware integer and are accepted anywhere a
    feed-forward mode is expected. ``Off`` disables the constant and velocity-table terms, ``Constant``
    enables only the constant term, ``VelocityTable`` enables only the
    velocity-table term, and ``ConstantAndVelocityTable`` enables both terms.
    """
    Off: "FeedForwardMode"
    Constant: "FeedForwardMode"
    VelocityTable: "FeedForwardMode"
    ConstantAndVelocityTable: "FeedForwardMode"
    def __repr__(self) -> str: ...

class FingertipSensorModel:
    """Fingertip sensor model reported by ``hand.fingertip_<finger>_status().get()``.

    Members compare equal to the firmware/C integer
    (``FingertipSensorModel.Thumb == 1``); ``Standard`` is a normal fingertip,
    ``Thumb`` a 40-point thumb node.
    """
    Standard: "FingertipSensorModel"
    Thumb: "FingertipSensorModel"
    def __repr__(self) -> str: ...

class FingertipSensorState:
    """Fingertip sensor runtime state reported by ``hand.fingertip_<finger>_status().get()``.

    Members compare equal to the firmware/C integer
    (``FingertipSensorState.Calibrating == 1``). ``Calibrating`` while a runtime
    calibration is in progress, ``Ready`` once complete.
    """
    Ready: "FingertipSensorState"
    Calibrating: "FingertipSensorState"
    def __repr__(self) -> str: ...

class FingertipSensorStatus:
    """A fingertip sensor's model + runtime state, returned by
    ``hand.fingertip_<finger>_status().get()``."""
    model: FingertipSensorModel
    state: FingertipSensorState
    def __repr__(self) -> str: ...

class FingertipStatusResource:
    """A finger's status accessor, from ``hand.fingertip_<finger>_status()``."""
    def get(self) -> FingertipSensorStatus:
        """Query the sensor's model and runtime state (blocking, up to ~1 s)."""
        ...
    def __repr__(self) -> str: ...

# Deprecated aliases, removed in 2026.11. Accessing them emits a DeprecationWarning.
TactileType: TypeAlias = FingertipSensorModel
TactileState: TypeAlias = FingertipSensorState
TactileStatus: TypeAlias = FingertipSensorStatus

class DeviceDiagnosticsResource:
    """Device-level diagnostics resource. get() returns list[20] of Optional[JointDiagnostics]; raises on disconnect, offline joints are None."""
    def get(self) -> List[Optional["JointDiagnostics"]]: ...
    def __repr__(self) -> str: ...

class DiagnosticStatus:
    """Diagnostics status for a single joint (ROS diagnostic_msgs/DiagnosticStatus style).

    level: 0=OK, 1=WARN, 2=ERROR
    name: joint label (e.g. 'thumb_j0')
    message: human-readable status string
    hardware_id: joint index as string
    values: list of (key, value) tuples with diagnostic key-value pairs
    """
    level: int
    name: str
    message: str
    hardware_id: str
    values: List[tuple]
    def __repr__(self) -> str: ...

class DiagnosticArray:
    """Diagnostics report for all joints (ROS diagnostic_msgs/DiagnosticArray style).

    status: list of DiagnosticStatus entries (one per online joint)
    """
    status: List[DiagnosticStatus]
    def __len__(self) -> int: ...
    def __repr__(self) -> str: ...

class OnlineJointsCountResource:
    """Device-level online joints count resource. `.get()` counts non-None diagnostics entries; raises on disconnect."""
    def get(self) -> int: ...
    def __repr__(self) -> str: ...

class WujiHand2(WujiHand2):
    """Hand-written extension methods for WujiHand2"""

    @property
    def device(self) -> WujiDevice: ...

    def handedness(self) -> HandednessResource: ...

    def control_mode(self) -> ControlModeResource:
        """Whole-hand control mode: one Mit or Position value for all 20 joints.

        In Position mode ``joint_command().publish().send(...)`` uses only each
        joint's position.
        """
        ...

    def position_command_low_pass_cutoff_frequency(self) -> PositionCommandLowPassCutoffFrequencyResource:
        """Whole-hand position-command low-pass cutoff frequency in Hz.

        Used for command upsampling in Position mode; does not filter MIT commands."""
        ...
    # No arg = all joints; joints=[0/1]*20 = flat-20 mask (by flat joint index,
    # nonzero = act on that joint). Single joint: hand.joint(k).enable()/disable()/clear_fault().
    def enable(self, joints: Optional[List[int]] = ...) -> None: ...
    def disable(self, joints: Optional[List[int]] = ...) -> None: ...
    def clear_fault(self, joints: Optional[List[int]] = ...) -> None: ...
    def emergency_stop(self) -> None: ...  # whole-hand safety action (no mask)
    def calibrate_fingertip(self) -> None:
        """Runtime-calibrate all 5 fingertip sensors to a zero baseline.

        ALL sensor surfaces MUST be unloaded (no contact force) when called.
        Whole-hand only; fail-fast — raises on the first offline finger. Success
        means every finger's command was enqueued, not that calibration finished;
        poll ``fingertip_<finger>_status().get()`` until
        ``state == FingertipSensorState.Ready`` to confirm.
        """
        ...
    @deprecated("WujiHand2.tactile_calibrate() is deprecated and will be removed in 2026.11; use calibrate_fingertip()")
    def tactile_calibrate(self) -> None: ...
    def fingertip_thumb_status(self) -> "FingertipStatusResource":
        """The thumb fingertip's status accessor; ``.get()`` takes no argument."""
        ...
    def fingertip_index_status(self) -> "FingertipStatusResource":
        """The index fingertip's status accessor; ``.get()`` takes no argument."""
        ...
    def fingertip_middle_status(self) -> "FingertipStatusResource":
        """The middle fingertip's status accessor; ``.get()`` takes no argument."""
        ...
    def fingertip_ring_status(self) -> "FingertipStatusResource":
        """The ring fingertip's status accessor; ``.get()`` takes no argument."""
        ...
    def fingertip_pinky_status(self) -> "FingertipStatusResource":
        """The pinky fingertip's status accessor; ``.get()`` takes no argument."""
        ...
    @deprecated("WujiHand2.tactile_status(finger) is deprecated and will be removed in 2026.11; use fingertip_<finger>_status().get()")
    def tactile_status(self, finger: str) -> TactileStatus: ...
    def fingertip_online(self) -> List[bool]:
        """Which fingers currently have a tactile module online (blocking).

        Returns five bools in thumb→index→middle→ring→pinky order. ``True``
        means that finger's tactile module is answering the hand's heartbeat
        right now. Live link state, not an installation flag: all
        ``False`` means no tactile module is online at this moment (none
        fitted, unplugged, or rebooting read the same), so
        ``any(hand.fingertip_online())`` answers "is any tactile module online
        now". Works with older firmware as well.
        """
        ...
    def fingertip_thumb_info(self) -> "FingertipInfoResource":
        """The thumb fingertip's info accessor; ``.get()`` takes no argument."""
        ...
    def fingertip_index_info(self) -> "FingertipInfoResource":
        """The index fingertip's info accessor; ``.get()`` takes no argument."""
        ...
    def fingertip_middle_info(self) -> "FingertipInfoResource":
        """The middle fingertip's info accessor; ``.get()`` takes no argument."""
        ...
    def fingertip_ring_info(self) -> "FingertipInfoResource":
        """The ring fingertip's info accessor; ``.get()`` takes no argument."""
        ...
    def fingertip_pinky_info(self) -> "FingertipInfoResource":
        """The pinky fingertip's info accessor; ``.get()`` takes no argument."""
        ...
    # Origin actions (joints=None = all via ROOT broadcast; [0/1]*20 = per-joint mask).
    def set_origin(self, joints: Optional[List[int]] = ...) -> None: ...
    def clear_origin(self, joints: Optional[List[int]] = ...) -> None: ...
    # Feed-forward control actions (joints=None = all joints; [0/1]*20 = per-joint
    # mask). Registered per-joint in firmware (no whole-hand handler), so even
    # joints=None fans out over per-joint P2P; requires firmware that supports
    # this feature.
    def set_constant(self, mode: Union[FeedForwardMode, int], joints: Optional[List[int]] = ...) -> None:
        """Set the feed-forward mode using a ``FeedForwardMode`` or integer 0..=3.

        Applied joint by joint in index order; stops at the first failing joint; earlier joints keep the new setting.
        """
        ...
    def set_position_table(self, on: bool, joints: Optional[List[int]] = ...) -> None:
        """Enable/disable position-indexed lookup-table feed-forward. RAM only.

        Applied joint by joint in index order; stops at the first failing joint; earlier joints keep the new setting.
        """
        ...
    # Resource accessors (whole-hand hand.X()):
    def effort_limit(self) -> DeviceEffortLimitResource: ...
    def mit_params(self) -> DeviceMitParamsResource: ...
    # Navigation:
    def joints(self) -> List[JointHandle]: ...
    def fingers(self) -> List[FingerHandle]: ...
    def joint(self, joint_index: int) -> JointHandle: ...
    def online_joints_count(self) -> OnlineJointsCountResource: ...

    @staticmethod
    def nid_to_joint_index(nid: int) -> int:
        """Convert a firmware stream NID (as carried by ``JointStateEntry.nid``)
        into the flat joint index (0..=19) used by command arrays and ``joint(idx)``.

        NID layout: ``bus * 5 + node`` with node 1..=4 the joints and node 5 the
        tactile slot, so joint nids are 1-4, 6-9, 11-14, 16-19, 21-24.

        Raises:
            WujiException: for tactile nids, ROOT (0), or out-of-range nids.
        """
        ...

    @staticmethod
    def joint_index_to_nid(joint_index: int) -> int:
        """Convert a flat joint index (0..=19) into the firmware stream NID.

        Raises:
            WujiException: for an index outside 0..=19.
        """
        ...

    @staticmethod
    def describe_error(code: int) -> Optional[ErrorInfo]:
        """Return a description of a device fault code as a ``dict``; ``None`` if unknown.

        Accepts the fault codes carried by ``JointDiagnosticsEntry.error_code_current``,
        the four severity fields of ``ErrorStatus``, ``ErrorLogEntry.error_code`` and
        ``Enc1/Enc2CalibResult.system_error_code``. The calibration-only
        ``enc1_error_code`` / ``enc2_error_code`` bytes are a different, unrelated
        enum and must not be passed here.

        Code ``0`` is not a fault code — it is the "no active error" value those
        fields carry when healthy, and it is deliberately absent from the catalog.
        ``describe_error(0)`` therefore returns ``None``; treat that as "nothing to
        report", **not** as an unknown fault. Only for a *non-zero* code returning
        ``None`` (SDK older than the firmware) surface the raw numeric code to the
        user rather than dropping it.

        The result is a ``dict``, not an object — read it by key
        (``info["severity"]``, not ``info.severity``); see the ``ErrorInfo``
        TypedDict for the keys and their types. Read ``severity`` and
        ``clear_policy`` rather than decoding the code's hex digits yourself.
        ``desc`` / ``cause`` / ``resolution`` are the firmware catalog's SDK-facing
        text and default to English.
        """
        ...

    def export_flash_logs(
        self, out_dir: Optional[str] = None
    ) -> dict[str, Any]:
        """Export the device's on-flash log history to a JSONL file (blocking).

        Writes ``flash_<serial>_<YYYY-MM-DD_HHMMSS>.jsonl`` into ``out_dir``, or
        into ``~/.wuji/logs`` — next to the SDK's own logs — when ``out_dir`` is
        ``None``. Returns ``{"path": str, "frames": int}``.
        """
        ...

# ────────────────────────────────────────────
# Retargeting
# ────────────────────────────────────────────

class HandModel:
    """Which Wuji hand model the qpos targets.

    Selects the builtin IK config (config + URDF).
    """
    WujiHand: "HandModel"
    WujiHand2: "HandModel"
    def __str__(self) -> str: ...
    def __repr__(self) -> str: ...

class RetargetSession:
    """Maps (21, 3) MediaPipe keypoints to a (20,) Wuji Hand joint-angle vector.

    Holds warm-start + low-pass-filter state across steps. Build one with
    `for_hand`; feed frames with `step`; clear state with `reset`.
    """

    @classmethod
    def for_hand(
        cls,
        hand_model: HandModel,
        side: Handedness,
    ) -> "RetargetSession":
        """Create a session for a hand model using its builtin tuning config.

        ``hand_model`` selects the builtin tuning config (config + URDF).

        Args:
            hand_model: a HandModel (HandModel.WujiHand / HandModel.WujiHand2).
            side: a Handedness (Handedness.Left / Handedness.Right).
        """
        ...

    def step(self, keypoints: npt.ArrayLike) -> npt.NDArray[np.float32]:
        """Retarget one frame.

        Args:
            keypoints: array-like of shape (21, 3) (or 63 elements row-major),
                MediaPipe landmark order.

        Returns:
            np.ndarray of shape (20,), float32 -- joint angles in firmware order.
        """
        ...

    def reset(self) -> None:
        """Reset warm-start and low-pass-filter state."""
        ...
