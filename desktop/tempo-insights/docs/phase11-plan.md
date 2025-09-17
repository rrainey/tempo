# Phase 11: Formation Review (D3) MVP - Detailed Design (Final)

## Overview
Phase 11 implements the interactive 3D visualization for formation skydiving analysis. This system will display multiple skydivers' positions relative to a designated "base" jumper, with real-time playback controls and performance metrics. The revised design incorporates the formal coordinate system definitions and emphasizes the Base Exit Frame as the primary reference for formation analysis.

## Coordinate System Architecture

Based on the coordinate frames document, the visualization pipeline will use:

1. **WGS-84 Geodetic** → **NED,DZ** → **Base Exit Frame** conversion pipeline
2. **Base Exit Frame** as the primary coordinate system for formation analysis
3. No need for ECI or ECEF conversions (as noted in the conversion table)
4. No geoid corrections applied (per clarification)

### Key Coordinate Systems:

- **WGS-84 Geodetic**: Raw GPS data from logs (lat/lon in degrees, altitude in meters)
- **NED,DZ**: Intermediate frame for efficient base jumper switching
- **Base Exit Frame**: Dynamic NED frame centered on base jumper's real-time position, with X-axis aligned to base jumper's ground track
- **Display Projections**: XY plane (God's eye) and XZ plane (Side view) of Base Exit Frame

## Task-by-Task Design

### Task 81: Data API
**Endpoint**: `/api/formations/[id]`

**Response Structure**:
```typescript
interface FormationData {
  id: string;
  startTime: Date;
  baseJumperId: string;
  jumpRunTrack_degTrue: number; // Ground track of base jumper at exit
  participants: ParticipantData[];
  dzElevation_m?: number; // For AGL calculations
}

interface ParticipantData {
  userId: string;
  jumpLogId: string;
  name: string;
  color: string; // For visualization
  isBase: boolean;
  isVisible: boolean; // From FormationParticipant.isVisibleToOthers
  timeSeries: TimeSeriesPoint[];
}

interface TimeSeriesPoint {
  timeOffset: number; // Seconds from formation start
  // WGS-84 position data
  location: {
    lat_deg: number;
    lon_deg: number;
    alt_m: number; // WGS-84 altitude (not MSL)
  };
  // Additional metrics
  baroAlt_ft: number; // From interpolated PENV data
  groundspeed_kmph?: number;
  groundtrack_degT?: number;
  verticalSpeed_mps?: number;
  normalizedFallRate_mph?: number; // Calibrated using density table
}
```

**Implementation Notes**:
- Query `FormationSkydive` with `FormationParticipant` joins
- Extract time series from each participant's `JumpLog`
- Use existing `KMLDataV1` structure from `dropkick-reader.ts` as basis
- Mock with realistic 4Hz GPS data for 2-4 jumpers initially
- Enforce visibility rules - exclude hidden participants unless viewer is that participant
- Store `baseJumperId` in database for persistence across sessions
- Formation window tightened to ±20s based on GNSS time accuracy

### Task 82: Projection Math
**Module**: `lib/formation/coordinates.ts`

**Core Conversion Function**:
```typescript
import { geodesy } from 'geodesy'; // npm package referenced in coordinate-frames.md

// Primary coordinate transformation with intermediate NED,DZ step
function wgs84ToNEDDZ(
  point: GeodeticCoordinates,
  dzCenter: GeodeticCoordinates
): Vector3 {
  // Convert to NED coordinates relative to DZ center
  // This intermediate step allows efficient base switching
  const p1 = new LatLon(point.lat_deg, point.lon_deg, point.alt_m);
  const center = new LatLon(dzCenter.lat_deg, dzCenter.lon_deg, dzCenter.alt_m);
  
  const bearing = center.bearingTo(p1);
  const distance = center.distanceTo(p1);
  const dAlt = point.alt_m - dzCenter.alt_m;
  
  const north = distance * Math.cos(bearing * Math.PI / 180);
  const east = distance * Math.sin(bearing * Math.PI / 180);
  const down = -dAlt;
  
  return { x: north, y: east, z: down };
}

function nedDZToBaseExitFrame(
  nedPos: Vector3,
  baseNEDPos: Vector3,
  baseGroundTrack_deg: number
): Vector3 {
  // Translate to base position
  const translated = {
    x: nedPos.x - baseNEDPos.x,
    y: nedPos.y - baseNEDPos.y,
    z: nedPos.z - baseNEDPos.z
  };
  
  // Rotate to align with base jumper's ground track
  const rotation = baseGroundTrack_deg * Math.PI / 180;
  const x = translated.x * Math.cos(rotation) + translated.y * Math.sin(rotation);
  const y = -translated.x * Math.sin(rotation) + translated.y * Math.cos(rotation);
  const z = translated.z;
  
  return { x, y, z }; // x: forward along ground track, y: right, z: down
}

// Apply to all participants at given time
function projectFormationAtTime(
  participants: ParticipantData[],
  timeOffset: number,
  baseJumperId: string,
  dzCenter: GeodeticCoordinates
): ProjectedPosition[] {
  // Find base jumper
  const base = participants.find(p => p.userId === baseJumperId);
  if (!base) throw new Error('Base jumper not found');
  
  // Interpolate base position and metrics at timeOffset
  const baseData = interpolatePosition(base.timeSeries, timeOffset);
  const baseNEDPos = wgs84ToNEDDZ(baseData.location, dzCenter);
  
  // Project all jumpers
  return participants.map(p => {
    const data = interpolatePosition(p.timeSeries, timeOffset);
    const nedPos = wgs84ToNEDDZ(data.location, dzCenter);
    const projected = nedDZToBaseExitFrame(
      nedPos, 
      baseNEDPos, 
      baseData.groundtrack_degT || 0
    );
    
    return {
      userId: p.userId,
      name: p.name,
      color: p.color,
      position: projected,
      isDataGap: data.isInterpolated, // Flag for visual indication
      metrics: {
        baroAlt_ft: data.baroAlt_ft,
        verticalSpeed_mps: data.verticalSpeed_mps,
        normalizedFallRate_mph: data.normalizedFallRate_mph,
        groundtrack_degT: data.groundtrack_degT,
        groundspeed_kmph: data.groundspeed_kmph
      }
    };
  });
}

// Helper: Interpolate position between samples
function interpolatePosition(
  timeSeries: TimeSeriesPoint[],
  timeOffset: number
): TimeSeriesPoint & { isInterpolated: boolean } {
  // Find surrounding samples
  let before = timeSeries[0];
  let after = timeSeries[timeSeries.length - 1];
  let isInterpolated = false;
  
  for (let i = 0; i < timeSeries.length - 1; i++) {
    if (timeSeries[i].timeOffset <= timeOffset && 
        timeSeries[i + 1].timeOffset > timeOffset) {
      before = timeSeries[i];
      after = timeSeries[i + 1];
      isInterpolated = (after.timeOffset - before.timeOffset) > 0.5; // Gap > 0.5s
      break;
    }
  }
  
  // Linear interpolation for all fields
  const t = (timeOffset - before.timeOffset) / 
            (after.timeOffset - before.timeOffset);
            
  return {
    timeOffset,
    location: {
      lat_deg: before.location.lat_deg + t * (after.location.lat_deg - before.location.lat_deg),
      lon_deg: before.location.lon_deg + t * (after.location.lon_deg - before.location.lon_deg),
      alt_m: before.location.alt_m + t * (after.location.alt_m - before.location.alt_m)
    },
    baroAlt_ft: before.baroAlt_ft + t * (after.baroAlt_ft - before.baroAlt_ft),
    groundtrack_degT: after.groundtrack_degT, // Use latest track
    groundspeed_kmph: after.groundspeed_kmph, // Use latest speed
    verticalSpeed_mps: before.verticalSpeed_mps, // Will be calculated separately
    normalizedFallRate_mph: before.normalizedFallRate_mph,
    isInterpolated
  };
}
```

**Fall Rate Calibration**:
```typescript
// Implement calibration table from coordinate-frames.md
const FALL_RATE_CALIBRATION = [
  { alt_ft: 20000, factor: 0.8107 },
  { alt_ft: 18000, factor: 0.8385 },
  // ... full table from document
  { alt_ft: 0, factor: 1.1107 }
];

function calibrateFallRate(
  verticalSpeed_mps: number,
  altitude_ft: number
): number {
  // Use interp1 to get calibration factor
  const factor = interp1(
    FALL_RATE_CALIBRATION.map(c => c.alt_ft),
    FALL_RATE_CALIBRATION.map(c => c.factor),
    altitude_ft
  );
  
  // Convert to mph and apply calibration
  const uncalibrated_mph = verticalSpeed_mps * 2.23694;
  return uncalibrated_mph / factor; // Normalized to 7000ft reference
}
```

### Task 83: Canvas Render Loop
**Component**: `components/formation/FormationViewer.tsx`

**Architecture** (Using Three.js):
```typescript
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';

interface FormationViewerState {
  currentTime: number;
  isPlaying: boolean;
  playbackSpeed: number; // 1x, 2x, 0.5x, etc.
  viewMode: 'godsEye' | 'side';
  showTrails: boolean;
  trailLength: number; // seconds
  baseJumperId: string; // Can be changed dynamically
}

const FormationViewer: React.FC<{formation: FormationData}> = ({formation}) => {
  const mountRef = useRef<HTMLDivElement>(null);
  const sceneRef = useRef<THREE.Scene>();
  const rendererRef = useRef<THREE.WebGLRenderer>();
  const cameraRef = useRef<THREE.PerspectiveCamera>();
  const jumperMeshes = useRef<Map<string, THREE.Mesh>>(new Map());
  
  const [state, setState] = useState<FormationViewerState>({
    currentTime: 0,
    isPlaying: false,
    playbackSpeed: 1,
    viewMode: 'godsEye',
    showTrails: true,
    trailLength: 3,
    baseJumperId: formation.baseJumperId
  });
  
  // Initialize Three.js scene
  useEffect(() => {
    if (!mountRef.current) return;
    
    // Scene setup
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x002233); // Theme background
    sceneRef.current = scene;
    
    // Camera
    const camera = new THREE.PerspectiveCamera(
      50, 
      mountRef.current.clientWidth / mountRef.current.clientHeight,
      0.1,
      10000
    );
    camera.position.set(0, 0, 200);
    cameraRef.current = camera;
    
    // Renderer
    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(mountRef.current.clientWidth, mountRef.current.clientHeight);
    mountRef.current.appendChild(renderer.domElement);
    rendererRef.current = renderer;
    
    // Controls
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    
    // Grid helper
    const gridHelper = new THREE.GridHelper(200, 20);
    gridHelper.rotation.x = Math.PI / 2; // Rotate for god's eye view
    scene.add(gridHelper);
    
    // Lighting
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    scene.add(ambientLight);
    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.4);
    directionalLight.position.set(100, 100, 50);
    scene.add(directionalLight);
    
    // Animation loop
    const animate = () => {
      requestAnimationFrame(animate);
      controls.update();
      renderer.render(scene, camera);
    };
    animate();
    
    return () => {
      mountRef.current?.removeChild(renderer.domElement);
      renderer.dispose();
    };
  }, []);
  
  // Update jumper positions
  useEffect(() => {
    if (!sceneRef.current) return;
    
    const positions = projectFormationAtTime(
      formation.participants,
      state.currentTime,
      state.baseJumperId,
      formation.dzCenter || {lat_deg: 0, lon_deg: 0, alt_m: 0}
    );
    
    positions.forEach(pos => {
      let mesh = jumperMeshes.current.get(pos.userId);
      
      if (!mesh) {
        // Create new jumper mesh
        const geometry = new THREE.SphereGeometry(2, 16, 16);
        const material = new THREE.MeshPhongMaterial({ 
          color: pos.color,
          transparent: true,
          opacity: pos.isDataGap ? 0.5 : 1.0 // Visual indication of gaps
        });
        mesh = new THREE.Mesh(geometry, material);
        sceneRef.current.add(mesh);
        jumperMeshes.current.set(pos.userId, mesh);
      }
      
      // Update position (convert meters to display units)
      mesh.position.set(
        pos.position.x,
        state.viewMode === 'godsEye' ? pos.position.y : -pos.position.z,
        state.viewMode === 'godsEye' ? 0 : pos.position.y
      );
      
      // Update opacity for data gaps
      (mesh.material as THREE.MeshPhongMaterial).opacity = 
        pos.isDataGap ? 0.5 : 1.0;
    });
    
    // Update trails if enabled
    if (state.showTrails) {
      updateTrails(positions, state.currentTime, state.trailLength);
    }
  }, [state.currentTime, state.viewMode, state.baseJumperId, formation]);
  
  // Playback animation
  useEffect(() => {
    if (!state.isPlaying) return;
    
    let lastTimestamp: number | null = null;
    let animationId: number;
    
    const animate = (timestamp: number) => {
      if (!lastTimestamp) lastTimestamp = timestamp;
      const deltaTime = (timestamp - lastTimestamp) / 1000;
      
      setState(prev => ({
        ...prev,
        currentTime: Math.min(
          prev.currentTime + (deltaTime * prev.playbackSpeed),
          getMaxTime(formation)
        )
      }));
      
      lastTimestamp = timestamp;
      animationId = requestAnimationFrame(animate);
    };
    
    animationId = requestAnimationFrame(animate);
    return () => cancelAnimationFrame(animationId);
  }, [state.isPlaying, state.playbackSpeed, formation]);
  
  // Handle base jumper change
  const handleBaseChange = (newBaseId: string) => {
    setState(prev => ({ ...prev, baseJumperId: newBaseId }));
    // Persist to database
    updateFormationBase(formation.id, newBaseId);
  };
  
  return (
    <Stack>
      <div ref={mountRef} style={{ width: '100%', height: '600px' }} />
      <PlaybackControls state={state} setState={setState} />
      <BaseSelector 
        participants={formation.participants}
        currentBase={state.baseJumperId}
        onChange={handleBaseChange}
      />
    </Stack>
  );
};
```

**D3 Rendering Functions** (Alternative if not using Three.js):
```typescript
function renderFormation(
  svg: d3.Selection,
  positions: ProjectedPosition[],
  viewMode: 'godsEye' | 'side'
) {
  // Set up scales based on view mode
  const xScale = d3.scaleLinear()
    .domain([-100, 100]) // meters
    .range([50, 750]);
    
  const yScale = d3.scaleLinear()
    .domain(viewMode === 'godsEye' ? [-100, 100] : [-50, 50])
    .range([550, 50]);
  
  // Select axis based on view mode
  const getX = (pos: Vector3) => viewMode === 'godsEye' ? pos.x : pos.y;
  const getY = (pos: Vector3) => viewMode === 'godsEye' ? pos.y : -pos.z;
  
  // Update jumper positions
  const jumpers = svg.selectAll('.jumper')
    .data(positions, d => d.userId);
    
  jumpers.enter()
    .append('circle')
    .attr('class', 'jumper')
    .attr('r', 8)
    .attr('fill', d => d.color)
    .attr('opacity', d => d.isDataGap ? 0.5 : 1.0) // Visual gap indication
    .merge(jumpers)
    .transition()
    .duration(100)
    .attr('cx', d => xScale(getX(d.position)))
    .attr('cy', d => yScale(getY(d.position)))
    .attr('opacity', d => d.isDataGap ? 0.5 : 1.0);
    
  jumpers.exit().remove();
  
  // Update grid and axes
  updateGridAndAxes(svg, xScale, yScale, viewMode);
}
```

### Task 84: Preset Views
**Implementation**:
```typescript
const VIEW_CONFIGURATIONS = {
  godsEye: {
    name: "God's Eye View",
    description: "Looking down from above",
    axes: { x: 'x', y: 'y' }, // Base Exit Frame axes
    labels: {
      x: 'Forward (ft)', // Along base jumper's ground track
      y: 'Right (ft)'
    },
    scale: {
      x: [-100, 100], // meters, will convert to feet for display
      y: [-100, 100]
    }
  },
  side: {
    name: "Side View",
    description: "Looking from the side",
    axes: { x: 'y', y: '-z' }, // Right and Up (negative down)
    labels: {
      x: 'Right (ft)',
      y: 'Altitude Difference (ft)'
    },
    scale: {
      x: [-100, 100],
      y: [-50, 50] // Smaller vertical range
    }
  }
};

// View toggle button
<ButtonGroup>
  <Button 
    variant={viewMode === 'godsEye' ? 'filled' : 'outline'}
    onClick={() => setState({...state, viewMode: 'godsEye'})}
  >
    God's Eye View
  </Button>
  <Button 
    variant={viewMode === 'side' ? 'filled' : 'outline'}
    onClick={() => setState({...state, viewMode: 'side'})}
  >
    Side View
  </Button>
</ButtonGroup>
```

### Task 85: Base Info Panel
**Component**: `components/formation/BaseInfoPanel.tsx`

**Implementation**:
```typescript
interface BaseInfoPanelProps {
  formation: FormationData;
  currentTime: number;
  baseJumperId: string; // Accept current base ID
}

const BaseInfoPanel: React.FC<BaseInfoPanelProps> = ({formation, currentTime, baseJumperId}) => {
  const base = formation.participants.find(p => p.userId === baseJumperId);
  if (!base) return null;
  
  // Get interpolated metrics at current time
  const currentMetrics = interpolatePosition(base.timeSeries, currentTime);
  
  // Calculate AGL if DZ elevation available
  const altitudeAGL = formation.dzElevation_m 
    ? (currentMetrics.location.alt_m - formation.dzElevation_m) * 3.28084
    : null;
  
  return (
    <Card>
      <Title>Base: {base.name}</Title>
      <Stack>
        <Text>
          Fall Rate: {currentMetrics.verticalSpeed_mps * 2.23694:.1f} mph
        </Text>
        <Text>
          Normalized Fall Rate: {currentMetrics.normalizedFallRate_mph:.1f} mph
        </Text>
        <Text>
          Altitude (Baro): {currentMetrics.baroAlt_ft:.0f} ft
        </Text>
        {altitudeAGL && (
          <Text>
            Altitude AGL: {altitudeAGL:.0f} ft
          </Text>
        )}
        <Text size="xs" color="dimmed">
          Ground Track: {currentMetrics.groundtrack_degT:.0f}°
        </Text>
      </Stack>
    </Card>
  );
};
```

### Task 86: Jumper List Panel
**Component**: `components/formation/JumperListPanel.tsx`

**Implementation**:
```typescript
interface JumperMetrics {
  userId: string;
  name: string;
  color: string;
  distanceToBase_ft: number;
  closureRate_fps: number;
  closureRate_mph: number;
  relativeAltitude_ft: number; // Positive = above base
  horizontalSeparation_ft: number;
}

function calculateJumperMetrics(
  jumperPos: Vector3,
  basePos: Vector3,
  jumperVel: Vector3,
  baseVel: Vector3
): Partial<JumperMetrics> {
  // Calculate 3D distance
  const dx = jumperPos.x - basePos.x;
  const dy = jumperPos.y - basePos.y;
  const dz = jumperPos.z - basePos.z;
  const distance3D_m = Math.sqrt(dx*dx + dy*dy + dz*dz);
  const horizontalDist_m = Math.sqrt(dx*dx + dy*dy);
  
  // Calculate relative velocity
  const relVel = {
    x: jumperVel.x - baseVel.x,
    y: jumperVel.y - baseVel.y,
    z: jumperVel.z - baseVel.z
  };
  
  // Closure rate (positive = approaching)
  const lineOfSight = {
    x: dx / distance3D_m,
    y: dy / distance3D_m,
    z: dz / distance3D_m
  };
  const closureRate_mps = -(relVel.x * lineOfSight.x + 
                            relVel.y * lineOfSight.y + 
                            relVel.z * lineOfSight.z);
  
  return {
    distanceToBase_ft: distance3D_m * 3.28084,
    horizontalSeparation_ft: horizontalDist_m * 3.28084,
    closureRate_fps: closureRate_mps * 3.28084,
    closureRate_mph: closureRate_mps * 2.23694,
    relativeAltitude_ft: -dz * 3.28084 // Negative dz = above base
  };
}

const JumperListPanel: React.FC<{...}> = ({formation, currentTime}) => {
  const metrics = calculateAllJumperMetrics(formation, currentTime);
  
  return (
    <Table>
      <thead>
        <tr>
          <th>Jumper</th>
          <th>Distance</th>
          <th>Closure Rate</th>
          <th>Relative Alt</th>
        </tr>
      </thead>
      <tbody>
        {metrics.map(m => (
          <tr key={m.userId}>
            <td>
              <Badge color={m.color}>{m.name}</Badge>
            </td>
            <td>{m.distanceToBase_ft.toFixed(0)} ft</td>
            <td>
              {m.closureRate_fps.toFixed(1)} fps
              ({m.closureRate_mph.toFixed(0)} mph)
            </td>
            <td>
              {m.relativeAltitude_ft > 0 ? '+' : ''}
              {m.relativeAltitude_ft.toFixed(0)} ft
            </td>
          </tr>
        ))}
      </tbody>
    </Table>
  );
};
```

## Data Flow Architecture

### 1. Formation Detection (Phase 9 Integration)
- Analysis worker identifies jumps within ±20s window (tightened from ±120s)
- Creates `FormationSkydive` record with participants
- Stores exit time and base jumper ID for persistence

### 2. Data Pipeline
```
Raw Log Files → DropkickReader → KMLDataV1 → TimeSeriesPoint → WGS-84 Positions
                                                                        ↓
Display ← D3 Render ← View Projection ← Base Exit Frame ← [Coordinate Transform]
```

### 3. Real-time Calculations
- 10-30 FPS render loop (based on data complexity)
- Interpolation between 4Hz GPS samples
- Smooth transitions using D3 transitions
- Velocity calculations using finite differences

**Velocity Calculations**:

```typescript
interface VelocityData {
  horizontal: Vector3;  // From GNSS VTG record
  vertical: number;     // From barometric altitude changes
}

// Extract velocity from time series data
function calculateVelocities(
  timeSeries: TimeSeriesPoint[]
): TimeSeriesPoint[] {
  return timeSeries.map((point, index) => {
    // Horizontal velocity from GNSS VTG data
    const horizontalVel = {
      x: (point.groundspeed_kmph || 0) * Math.cos(point.groundtrack_degT * Math.PI / 180) / 3.6,
      y: (point.groundspeed_kmph || 0) * Math.sin(point.groundtrack_degT * Math.PI / 180) / 3.6,
      z: 0
    };
    
    // Vertical velocity from barometric altitude changes
    let verticalVel = 0;
    if (index > 0) {
      const dt = point.timeOffset - timeSeries[index - 1].timeOffset;
      const dAlt = (point.baroAlt_ft - timeSeries[index - 1].baroAlt_ft) * 0.3048;
      verticalVel = dAlt / dt; // m/s
    }
    
    return {
      ...point,
      horizontalVelocity: horizontalVel,
      verticalSpeed_mps: verticalVel
    };
  });
}
```

## Integration with Existing Code

### From `dropkick-reader.ts`:
- Use `KMLDataV1` structure as basis for time series
- Leverage `DropkickReader.onClose()` post-processing approach
- Apply barometric altitude interpolation technique

### From `dropkick-tools.ts`:
- Use `interp1` for smooth interpolation between samples
- Apply unit conversion functions (meters/feet, etc.)
- Integrate `plottableValuesFromSamples` approach for derived metrics

### From `coordinate-frames.md`:
- Implement fall rate calibration table
- Use Base Exit Frame as primary reference
- Apply proper altitude handling (WGS-84 vs MSL)

## Technical Considerations

### Performance Optimizations:
1. **Data Reduction**: 
   - Downsample trails to every 0.5s for display
   - Pre-calculate static metrics during data load
   - Use React.memo for panel components

2. **Smooth Playback**:
   - Use requestAnimationFrame for consistent timing
   - Buffer future positions for interpolation
   - Three.js handles rendering optimizations automatically

3. **Memory Management**:
   - Limit trail history to last 10-20 seconds
   - Use typed arrays for position data where possible
   - Properly dispose Three.js objects when switching formations

### Accuracy Considerations:
1. **GPS vs Barometric Altitude**:
   - Display barometric altitude as primary
   - No geoid corrections applied
   - Note potential discrepancies in UI if significant

2. **Coordinate Precision**:
   - Maintain full precision in calculations
   - Round only for display
   - Account for GPS uncertainty (~3m horizontal, ~5m vertical)

3. **Velocity Handling**:
   - Horizontal velocity from GNSS VTG records (ground track and speed)
   - Vertical velocity from PENV barometric samples (2 sample difference)
   - Mark interpolated gaps visually

## Testing Strategy

### Unit Tests:
```typescript
describe('Coordinate Transformations', () => {
  it('should correctly transform WGS-84 to Base Exit Frame', () => {
    // Test with known positions
  });
  
  it('should apply fall rate calibration correctly', () => {
    // Test against table values
  });
});

describe('Relative Metrics', () => {
  it('should calculate closure rates accurately', () => {
    // Test approaching, receding, and lateral motion
  });
});
```

### Integration Tests:
- Mock formation with 4 jumpers in diamond pattern
- Verify view switching maintains relative positions
- Test playback controls and time synchronization

### Visual Validation:
- Compare against known formation videos
- Verify spatial relationships look correct
- Test with edge cases (very close/far jumpers)

## MVP Implementation Priority

1. **Week 1**: Core coordinate math and static visualization
   - Implement WGS-84 to Base Exit Frame conversion
   - Create static D3 visualization at t=0
   - Basic view switching

2. **Week 2**: Animation and playback
   - Add playback controls
   - Implement smooth interpolation
   - Add trail visualization

3. **Week 3**: Information panels and polish
   - Base info panel with live metrics
   - Jumper list with relative calculations
   - Performance optimizations

This revised design fully integrates the coordinate system architecture from the updated document and provides a clear path for implementing formation visualization using the Base Exit Frame as the primary reference.