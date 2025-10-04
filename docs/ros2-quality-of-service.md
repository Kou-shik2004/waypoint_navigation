# ROS2 Quality of Service (QoS)

## Overview

Quality of Service (QoS) policies in ROS 2 allow you to tune communication between nodes for different network conditions and application requirements. With the right QoS settings, ROS 2 can be as reliable as TCP or as best-effort as UDP, with many states in between.

Unlike ROS 1 (which primarily used TCP), ROS 2 benefits from the flexibility of the underlying DDS transport, making it suitable for:
- Lossy wireless networks (best-effort policy)
- Real-time computing systems (appropriate deadline/liveliness policies)
- Critical data transmission (reliable policy)

## QoS Policies

### History
Controls how many messages to keep in the queue.

- **KEEP_LAST**: Store up to N samples (configurable via depth)
- **KEEP_ALL**: Store all samples (subject to resource limits)

```python
from rclpy.qos import QoSProfile, HistoryPolicy

qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)
```

### Depth
Queue size - only used when history is KEEP_LAST.

```python
qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10  # Keep last 10 messages
)
```

### Reliability
Determines delivery guarantees.

- **BEST_EFFORT**: Attempt delivery but may lose messages (faster, like UDP)
- **RELIABLE**: Guarantee delivery, retry if needed (slower, like TCP)

```python
from rclpy.qos import ReliabilityPolicy

qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE
)
```

### Durability
Controls message persistence for late-joining subscribers.

- **VOLATILE**: No persistence, only new messages
- **TRANSIENT_LOCAL**: Publisher persists messages for late-joiners (like ROS 1 latching)

```python
from rclpy.qos import DurabilityPolicy

qos = QoSProfile(
    durability=DurabilityPolicy.TRANSIENT_LOCAL
)
```

### Deadline
Expected maximum time between messages.

```python
from rclpy.duration import Duration

qos = QoSProfile(
    deadline=Duration(seconds=1)  # Expect message every second
)
```

### Lifespan
Maximum age of a message before it's considered stale.

```python
qos = QoSProfile(
    lifespan=Duration(seconds=5)  # Discard messages older than 5 seconds
)
```

### Liveliness
How publishers indicate they're still alive.

- **AUTOMATIC**: System tracks liveliness automatically
- **MANUAL_BY_TOPIC**: Publisher must manually assert liveliness

```python
from rclpy.qos import LivelinessPolicy

qos = QoSProfile(
    liveliness=LivelinessPolicy.AUTOMATIC,
    liveliness_lease_duration=Duration(seconds=2)
)
```

## Predefined QoS Profiles

ROS 2 provides preset profiles for common use cases:

### Sensor Data
For high-frequency sensor data where latest value matters most.

```python
from rclpy.qos import qos_profile_sensor_data

# Equivalent to:
# - History: KEEP_LAST
# - Depth: 5
# - Reliability: BEST_EFFORT
# - Durability: VOLATILE

publisher = self.create_publisher(
    LaserScan,
    '/scan',
    qos_profile_sensor_data
)
```

### Services Default
For service communications.

```python
from rclpy.qos import qos_profile_services_default

# Equivalent to:
# - History: KEEP_LAST
# - Depth: 10
# - Reliability: RELIABLE
# - Durability: VOLATILE
```

### Parameters
For parameter services (higher depth than regular services).

```python
from rclpy.qos import qos_profile_parameters

# Equivalent to:
# - History: KEEP_LAST
# - Depth: 1000
# - Reliability: RELIABLE
# - Durability: VOLATILE
```

### System Default
Uses RMW implementation defaults.

```python
from rclpy.qos import qos_profile_system_default
```

## Creating Custom QoS Profiles

### Method 1: From Scratch

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

custom_qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE
)

publisher = self.create_publisher(
    String,
    'topic',
    custom_qos
)
```

### Method 2: Modify Preset Profile

```python
from rclpy.qos import qos_profile_sensor_data

custom_qos = qos_profile_sensor_data
custom_qos.depth = 20
custom_qos.reliability = ReliabilityPolicy.RELIABLE
```

### Method 3: Using QoSPresetProfiles

```python
from rclpy.qos import QoSPresetProfiles

qos = QoSPresetProfiles.SENSOR_DATA.value
```

## QoS Compatibility

Publishers offer a QoS profile, subscribers request a QoS profile. Connection is made only if profiles are compatible.

### Reliability Compatibility

| Publisher | Subscription | Compatible | Notes |
|-----------|--------------|------------|-------|
| Best effort | Best effort | ✓ | Both accept message loss |
| Best effort | Reliable | ✗ | Sub needs guarantee pub can't provide |
| Reliable | Best effort | ✓ | Pub provides more than sub needs |
| Reliable | Reliable | ✓ | Both reliable |

### Durability Compatibility

| Publisher | Subscription | Compatible | Result |
|-----------|--------------|------------|--------|
| Volatile | Volatile | ✓ | New messages only |
| Volatile | Transient local | ✗ | No communication |
| Transient local | Volatile | ✓ | New messages only |
| Transient local | Transient local | ✓ | New and old messages |

### Deadline Compatibility

| Publisher | Subscription | Compatible |
|-----------|--------------|------------|
| Default | Default | ✓ |
| Default | x | ✗ |
| x | Default | ✓ |
| x | y (y > x) | ✓ |
| x | y (y < x) | ✗ |

Publisher's deadline must be <= Subscriber's deadline.

### Liveliness Compatibility

| Publisher | Subscription | Compatible |
|-----------|--------------|------------|
| Automatic | Automatic | ✓ |
| Automatic | Manual by topic | ✗ |
| Manual by topic | Automatic | ✓ |
| Manual by topic | Manual by topic | ✓ |

## Using QoS in Python

### Publisher with QoS

```python
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.publisher = self.create_publisher(
            String,
            'topic',
            qos
        )
```

### Subscription with QoS

```python
class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.callback,
            qos
        )
    
    def callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
```

### Service with QoS

```python
from rclpy.qos import qos_profile_services_default

service = self.create_service(
    AddTwoInts,
    'add_two_ints',
    self.service_callback,
    qos_profile=qos_profile_services_default
)
```

## QoS Events

You can subscribe to QoS events to monitor communication health.

### Available Events

**Publisher Events:**
- Offered deadline missed
- Liveliness lost
- Offered incompatible QoS

**Subscription Events:**
- Requested deadline missed
- Liveliness changed
- Requested incompatible QoS

### Example: Monitoring Incompatible QoS

```python
from rclpy.qos import QoSProfile
from rclpy.event_handler import PublisherEventCallbacks, SubscriptionEventCallbacks
from rclpy.qos_event import QoSPublisherEventType, QoSSubscriptionEventType

class MonitoredPublisher(Node):
    def __init__(self):
        super().__init__('monitored_publisher')
        
        event_callbacks = PublisherEventCallbacks(
            incompatible_qos=self.incompatible_qos_callback
        )
        
        self.publisher = self.create_publisher(
            String,
            'topic',
            QoSProfile(depth=10),
            event_callbacks=event_callbacks
        )
    
    def incompatible_qos_callback(self, event):
        self.get_logger().warning(
            f'Incompatible QoS: {event.total_count} incompatible connections'
        )
```

### Example: Monitoring Deadline Misses

```python
class DeadlineMonitor(Node):
    def __init__(self):
        super().__init__('deadline_monitor')
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            deadline=Duration(seconds=1)
        )
        
        event_callbacks = SubscriptionEventCallbacks(
            deadline=self.deadline_callback
        )
        
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.msg_callback,
            qos,
            event_callbacks=event_callbacks
        )
    
    def deadline_callback(self, event):
        self.get_logger().warning(
            f'Deadline missed! Total count: {event.total_count}'
        )
    
    def msg_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
```

### Example: Monitoring Liveliness

```python
class LivelinessMonitor(Node):
    def __init__(self):
        super().__init__('liveliness_monitor')
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            liveliness=LivelinessPolicy.AUTOMATIC,
            liveliness_lease_duration=Duration(seconds=2)
        )
        
        event_callbacks = SubscriptionEventCallbacks(
            liveliness=self.liveliness_callback
        )
        
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.msg_callback,
            qos,
            event_callbacks=event_callbacks
        )
    
    def liveliness_callback(self, event):
        if event.alive_count > 0:
            self.get_logger().info('Publisher is alive')
        else:
            self.get_logger().warning('Publisher is not alive')
    
    def msg_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
```

## Common Use Cases

### High-Frequency Sensor Data (Lidar, Camera)

```python
from rclpy.qos import qos_profile_sensor_data

publisher = self.create_publisher(
    LaserScan,
    '/scan',
    qos_profile_sensor_data  # Best effort, latest value
)
```

**Why this profile?**
- **Best effort**: Speed over reliability (occasional loss acceptable)
- **Keep last 5**: Only recent data matters
- **Volatile**: No need to persist old sensor readings

### Critical Control Commands

```python
qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    deadline=Duration(milliseconds=100)  # Must receive within 100ms
)

publisher = self.create_publisher(Twist, '/cmd_vel', qos)
```

**Why this profile?**
- **Reliable**: Every command must arrive
- **Deadline**: Detect communication delays
- **Keep last 10**: Buffer for processing spikes

### Map Data (Latched Topic)

```python
qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

publisher = self.create_publisher(OccupancyGrid, '/map', qos)
```

**Why this profile?**
- **Transient local**: Late-joiners receive last published map
- **Reliable**: Map data must not be lost
- **Keep last 1**: Only current map needed

### Robot State Publishing

```python
qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    deadline=Duration(milliseconds=50),
    liveliness=LivelinessPolicy.AUTOMATIC,
    liveliness_lease_duration=Duration(milliseconds=100)
)

publisher = self.create_publisher(JointState, '/joint_states', qos)
```

**Why this profile?**
- **Reliable**: State updates must arrive
- **Deadline**: Detect stale state
- **Liveliness**: Monitor publisher health

### Parameter Updates

```python
from rclpy.qos import qos_profile_parameters

# Use built-in parameter profile
# - Reliable
# - Keep last 1000
# - Volatile
```

### Best-Effort Streaming (Video)

```python
qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    lifespan=Duration(milliseconds=100)  # Drop frames older than 100ms
)

publisher = self.create_publisher(Image, '/camera/image', qos)
```

**Why this profile?**
- **Best effort**: Speed over completeness
- **Keep last 1**: Only latest frame matters
- **Lifespan**: Drop stale frames

## Debugging QoS Issues

### Check QoS Settings of Running Nodes

```bash
# List all topics with their QoS settings
ros2 topic info /topic_name --verbose

# Example output shows:
# Publishers: 1
#   Node: /publisher_node
#   QoS Profile:
#     Reliability: RELIABLE
#     History: KEEP_LAST
#     Depth: 10
#     Durability: VOLATILE
```

### Common Issues and Solutions

#### Issue: No Communication Between Nodes

**Symptom:** Publisher and subscriber both running, but no messages received.

**Diagnosis:**
```bash
ros2 topic info /topic_name --verbose
```

**Common causes:**
1. **QoS incompatibility** - Reliability or durability mismatch
2. **Type mismatch** - Different message types
3. **Namespace issues** - Topics in different namespaces

**Solution:**
```python
# Make subscriber compatible with publisher
# Option 1: Match publisher QoS exactly
qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,  # Match publisher
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# Option 2: Use more permissive subscriber QoS
qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,  # Can receive from any publisher
    durability=DurabilityPolicy.VOLATILE,       # Can receive from any publisher
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)
```

#### Issue: Messages Being Dropped

**Symptom:** Some messages not received, even with reliable QoS.

**Possible causes:**
1. **Queue overflow** - Depth too small
2. **Lifespan expired** - Messages too old
3. **Network issues** - Even reliable has limits

**Solution:**
```python
# Increase queue depth
qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=100  # Increase from default
)

# Or use KEEP_ALL
qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_ALL  # Keep all messages (resource limited)
)
```

#### Issue: High Latency

**Symptom:** Messages arrive with significant delay.

**Causes:**
1. **Reliable QoS** - Retransmissions add latency
2. **Large queue depth** - Old messages processed first
3. **Network congestion**

**Solution:**
```python
# For latency-sensitive applications
qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,  # Skip retransmissions
    history=HistoryPolicy.KEEP_LAST,
    depth=1  # Only latest message
)
```

#### Issue: Late-Joiner Not Receiving Data

**Symptom:** Subscriber started after publisher, missing initial messages.

**Solution:**
```python
# Publisher must use TRANSIENT_LOCAL
qos = QoSProfile(
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

# Subscriber must also request TRANSIENT_LOCAL
subscriber_qos = QoSProfile(
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)
```

#### Issue: Deadline Missed Warnings

**Symptom:** Constant deadline missed events.

**Diagnosis:**
```python
# Add callback to track frequency
def deadline_callback(self, event):
    self.get_logger().warning(
        f'Deadline missed! Total: {event.total_count}, '
        f'Total count change: {event.total_count_change}'
    )
```

**Solutions:**
```python
# Option 1: Increase deadline duration
qos = QoSProfile(
    deadline=Duration(seconds=2)  # Was 1 second
)

# Option 2: Increase publisher frequency
self.timer = self.create_timer(0.5, self.publish_callback)  # Was 1.0

# Option 3: Remove deadline if not critical
qos = QoSProfile(
    # No deadline specified
)
```

## Performance Considerations

### Best Effort vs Reliable

**Best Effort:**
- ✓ Lower latency
- ✓ Lower CPU usage
- ✓ Lower bandwidth
- ✗ Message loss possible

**Reliable:**
- ✓ No message loss
- ✗ Higher latency (retransmissions)
- ✗ Higher CPU usage
- ✗ Higher bandwidth (acknowledgments)

### Queue Depth Impact

```python
# Small depth (1-10)
# + Lower memory usage
# + Lower latency
# - More message drops if consumer slow

# Large depth (100+)
# + Fewer message drops
# - Higher memory usage
# - Potentially stale data processed
```

### Durability Impact

```python
# VOLATILE
# + Lower memory usage
# + Simpler implementation
# - Late-joiners miss data

# TRANSIENT_LOCAL
# + Late-joiners receive last message
# - Higher memory usage
# - Publisher keeps message history
```

## Best Practices

1. **Match QoS to Requirements**
   - Sensor data → Best effort, low depth
   - Commands → Reliable, moderate depth
   - Configuration → Reliable, transient local

2. **Start Conservative, Optimize Later**
   ```python
   # Start with reliable
   qos = QoSProfile(
       reliability=ReliabilityPolicy.RELIABLE,
       history=HistoryPolicy.KEEP_LAST,
       depth=10
   )
   
   # Profile performance, then optimize
   # Switch to best effort if message loss acceptable
   ```

3. **Monitor QoS Events**
   ```python
   # Always monitor in production
   event_callbacks = SubscriptionEventCallbacks(
       deadline=self.deadline_callback,
       liveliness=self.liveliness_callback,
       incompatible_qos=self.incompatible_qos_callback
   )
   ```

4. **Document QoS Choices**
   ```python
   # BAD: No explanation
   qos = QoSProfile(depth=10)
   
   # GOOD: Clear reasoning
   qos = QoSProfile(
       reliability=ReliabilityPolicy.RELIABLE,  # Safety critical
       history=HistoryPolicy.KEEP_LAST,
       depth=10,  # Handle 100ms of messages at 100Hz
       deadline=Duration(milliseconds=50)  # Detect delays > 50ms
   )
   ```

5. **Test QoS Compatibility**
   ```bash
   # Before deployment, verify compatibility
   ros2 topic info /topic_name --verbose
   
   # Test late-joiner behavior
   # 1. Start publisher
   # 2. Publish messages
   # 3. Start subscriber (should receive based on durability)
   ```

6. **Use Preset Profiles When Appropriate**
   ```python
   # Don't reinvent common patterns
   from rclpy.qos import qos_profile_sensor_data
   
   # Use preset for sensor data
   publisher = self.create_publisher(
       LaserScan, '/scan', qos_profile_sensor_data
   )
   ```

7. **Consider Network Conditions**
   ```python
   # WiFi or unreliable network
   qos = QoSProfile(
       reliability=ReliabilityPolicy.BEST_EFFORT,  # Avoid retransmit storms
       history=HistoryPolicy.KEEP_LAST,
       depth=5  # Small buffer
   )
   
   # Wired or reliable network
   qos = QoSProfile(
       reliability=ReliabilityPolicy.RELIABLE,
       history=HistoryPolicy.KEEP_LAST,
       depth=100  # Larger buffer acceptable
   )
   ```

## QoS API Reference

### QoSProfile Class

```python
from rclpy.qos import QoSProfile

QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    deadline=Duration(seconds=0),  # 0 = infinite
    lifespan=Duration(seconds=0),  # 0 = infinite
    liveliness=LivelinessPolicy.AUTOMATIC,
    liveliness_lease_duration=Duration(seconds=0),  # 0 = infinite
    avoid_ros_namespace_conventions=False
)
```

### Enums

```python
from rclpy.qos import (
    HistoryPolicy,
    ReliabilityPolicy,
    DurabilityPolicy,
    LivelinessPolicy
)

# HistoryPolicy
HistoryPolicy.KEEP_LAST
HistoryPolicy.KEEP_ALL

# ReliabilityPolicy
ReliabilityPolicy.BEST_EFFORT
ReliabilityPolicy.RELIABLE

# DurabilityPolicy
DurabilityPolicy.VOLATILE
DurabilityPolicy.TRANSIENT_LOCAL

# LivelinessPolicy
LivelinessPolicy.AUTOMATIC
LivelinessPolicy.MANUAL_BY_TOPIC
```

### Preset Profiles

```python
from rclpy.qos import (
    qos_profile_sensor_data,
    qos_profile_services_default,
    qos_profile_parameters,
    qos_profile_system_default
)
```

### Duration Class

```python
from rclpy.duration import Duration

# Create duration
d1 = Duration(seconds=1)
d2 = Duration(seconds=1, nanoseconds=500000000)
d3 = Duration(nanoseconds=1000000)  # 1 millisecond

# Common durations
Duration(milliseconds=100)
Duration(seconds=5)
Duration(minutes=1)
```

## Summary

QoS in ROS 2 provides fine-grained control over communication behavior:

- **Reliability**: Trade off speed vs guarantee
- **Durability**: Control message persistence
- **History**: Manage queue behavior
- **Deadline**: Detect stale data
- **Liveliness**: Monitor connection health

Choose QoS settings based on:
1. Application requirements (safety, real-time, etc.)
2. Network conditions (WiFi, wired, bandwidth)
3. Data characteristics (frequency, size, importance)
4. Resource constraints (memory, CPU)

Always test QoS compatibility between publishers and subscribers, and monitor QoS events in production systems.
