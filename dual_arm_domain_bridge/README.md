# <p align="center">dual_arm_domain_bridge — Cross-Domain Bridge Layer (Plan B: domain isolation)</p>

## What This Package Does

<p align="justify">
The stock drivers of both HIWIN arms assume a <b> single arm </b> : the joint names are uniformly <code>joint_1</code> ~ <code>joint_6</code> (no prefix), states are always published on <code>/joint_states</code> , and the controller is always <code>/joint_trajectory_controller</code> . If the two arms were placed directly into the same ROS 2 network, the joint names and topics would inevitably collide.
</p>

<p align="justify">
This package adopts a <b> domain-isolation </b> solution: <b> the arm side runs the stock launch files entirely unmodified </b> , each confined to an independent ROS_DOMAIN (Arm A on D20, Arm B on D30) and mutually invisible; the host (D10) transports data across domains through this bridge layer, prefixing the joint names on the uplink. The stock drivers and the interface contract require no modification whatsoever.
</p>

## Composition

<p align="justify">
Two executables + one launch file:
</p>

<table width="100%">
  <thead>
    <tr>
      <th align="justify">Program</th>
      <th align="justify">Direction</th>
      <th align="justify">Role</th>
      <th align="justify">Notes</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td align="justify"><code>joint_state_uplink_bridge</code></td>
      <td align="justify">Uplink (the "eyes")</td>
      <td align="justify">Subscribes to the <code>/joint_states</code> of D20 / D30, prefixes each, merges them, and publishes to D10</td>
      <td align="justify"><b> Mandatory </b> ; otherwise the host cannot see the arms</td>
    </tr>
    <tr>
      <td align="justify"><code>trajectory_downlink_endpoint_relay</code></td>
      <td align="justify">Downlink (the "hands")</td>
      <td align="justify">Transparently relays the Execute and Stop commands from RViz / MoveIt to each arm</td>
      <td align="justify">The failure semantics are identical to a direct connection</td>
    </tr>
  </tbody>
</table>

<p align="justify">
Both executables accept the positional arguments <code>[host_d armA_d armB_d]</code> to specify the domains.
</p>

> <div align="justify"><b> Important </b> : the domain is hard-coded internally via <code>set_domain_id</code> and <b> does not honor the terminal's <code>ROS_DOMAIN_ID</code> environment variable </b> . <code>export ROS_DOMAIN_ID=...</code> has no effect on this bridge layer; to change a domain, always use the launch arguments or the positional arguments.</div>

## Prerequisites (Interface Contract)

<p align="justify">
The arm-side driver must satisfy the following interface (the HIWIN stock driver already does, requiring no changes):
</p>

<ul>
  <li align="justify" style="margin-bottom: 8px;">Joint names: <code>joint_1</code> ~ <code>joint_6</code> , no prefix, in radians</li>
  <li align="justify" style="margin-bottom: 8px;">Trajectory interface: <code>/joint_trajectory_controller/follow_joint_trajectory</code></li>
  <li align="justify" style="margin-bottom: 8px;">State publishing: <code>/joint_states</code></li>
</ul>

## Quick Start

### Bridge Layer (One Command)

```bash
# Use the default domains (host=10 / armA=20 / armB=30)
ros2 launch dual_arm_domain_bridge bridge_relay.launch.py

# When the arm domains differ from the defaults, override them via arguments (each of the three can be set individually)
ros2 launch dual_arm_domain_bridge bridge_relay.launch.py \
    host_domain:=10 arm_a_domain:=20 arm_b_domain:=30
```

<table width="100%">
  <thead>
    <tr>
      <th align="justify">launch argument</th>
      <th align="justify">Meaning</th>
      <th align="justify">Default</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td align="justify"><code>host_domain</code></td>
      <td align="justify">Host (planning side) domain</td>
      <td align="justify">10</td>
    </tr>
    <tr>
      <td align="justify"><code>arm_a_domain</code></td>
      <td align="justify">Arm A (RA610-1476) domain</td>
      <td align="justify">20</td>
    </tr>
    <tr>
      <td align="justify"><code>arm_b_domain</code></td>
      <td align="justify">Arm B (RA605-710) domain</td>
      <td align="justify">30</td>
    </tr>
  </tbody>
</table>

### Full-System Startup Order

<p align="justify">
Start in the order ①→④; <b> the order must not be reversed </b> (the arm side must already be online when the bridge layer starts):
</p>

```
① Arm A (RA610-1476): start on Domain 20 following the driver procedure provided by HIWIN
② Arm B (RA605-710) : start on Domain 30 following the driver procedure provided by HIWIN
③ Bridge layer:
   ros2 launch dual_arm_domain_bridge bridge_relay.launch.py
   (if the arms' actual domains are not 20 / 30, adjust accordingly via arm_a_domain / arm_b_domain)
④ Host (D10):
   ros2 launch hiwin_dual_arm brain.launch.py launch_rviz:=true
   (do not launch the merger again — joint_states merging is already handled by the uplink bridge; launching it again causes double merging)
```

### Post-Startup Verification

<p align="justify">
On the host (D10), confirm that the uplink is functioning:
</p>

```bash
ros2 topic echo /joint_states --once
```

<p align="justify">
You should see <b> 12 joints </b> (6 axes per arm, with names already prefixed). If you see only 6, or no output at all, refer to Troubleshooting below.
</p>

## Troubleshooting

<table width="100%">
  <thead>
    <tr>
      <th align="justify">Symptom</th>
      <th align="justify">Cause</th>
      <th align="justify">Remedy</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td align="justify">The host cannot see <code>/joint_states</code></td>
      <td align="justify">The bridge layer is not running, or the domain arguments do not match the arms' actual domains</td>
      <td align="justify">Confirm that step ③ is running; verify that <code>arm_a_domain</code> / <code>arm_b_domain</code> agree with the arm side</td>
    </tr>
    <tr>
      <td align="justify">Changing <code>ROS_DOMAIN_ID</code> has no effect</td>
      <td align="justify">This bridge layer does not read environment variables (by design)</td>
      <td align="justify">Use the launch arguments or positional arguments instead</td>
    </tr>
    <tr>
      <td align="justify">Only one arm's 6 joints are visible</td>
      <td align="justify">The other arm's driver is not running, or its domain is wrong</td>
      <td align="justify">Return to steps ①② and check that arm</td>
    </tr>
  </tbody>
</table>
