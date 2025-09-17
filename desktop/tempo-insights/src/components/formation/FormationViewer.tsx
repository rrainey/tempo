// components/formation/FormationViewer.tsx
import React, { useEffect, useRef, useState, useCallback } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';
import { Stack, Select, Button, Group, Badge, Slider, Text } from '@mantine/core';
import { IconPlayerPlay, IconPlayerPause, IconPlayerSkipForward } from '@tabler/icons-react';
import { projectFormationAtTime } from '../../lib/formation/coordinates';
import type { ParticipantData, ProjectedPosition } from '../../lib/formation/coordinates';
import type { GeodeticCoordinates } from '../../lib/formation/types';

interface FormationData {
  id: string;
  startTime: Date;
  baseJumperId: string;
  jumpRunTrack_degTrue: number;
  participants: ParticipantData[];
  dzElevation_m?: number;
}

interface FormationViewerState {
  currentTime: number;
  isPlaying: boolean;
  playbackSpeed: number;
  viewMode: 'godsEye' | 'side';
  showTrails: boolean;
  trailLength: number;
  baseJumperId: string;
}

interface FormationViewerProps {
  formation: FormationData;
  dzCenter: GeodeticCoordinates;
  onBaseChange?: (newBaseId: string) => void;
}

export const FormationViewer: React.FC<FormationViewerProps> = ({ 
  formation, 
  dzCenter,
  onBaseChange 
}) => {
  const mountRef = useRef<HTMLDivElement>(null);
  const sceneRef = useRef<THREE.Scene>();
  const rendererRef = useRef<THREE.WebGLRenderer>();
  const cameraRef = useRef<THREE.PerspectiveCamera>();
  const controlsRef = useRef<OrbitControls>();
  const jumperMeshes = useRef<Map<string, THREE.Mesh>>(new Map());
  const trailLines = useRef<Map<string, THREE.Line>>(new Map());
  const frameRef = useRef<number>();

  const [state, setState] = useState<FormationViewerState>({
    currentTime: 0,
    isPlaying: false,
    playbackSpeed: 1,
    viewMode: 'godsEye',
    showTrails: true,
    trailLength: 3,
    baseJumperId: formation.baseJumperId
  });

  // Calculate max time from formation data
  const getMaxTime = useCallback(() => {
    return Math.max(
      ...formation.participants.flatMap(p => 
        p.timeSeries.map(ts => ts.timeOffset)
      )
    );
  }, [formation]);

  // Initialize Three.js scene
  useEffect(() => {
    if (!mountRef.current) return;

    // Scene setup
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x002233); // Theme background
    scene.fog = new THREE.Fog(0x002233, 200, 1000);
    sceneRef.current = scene;

    // Camera
    const width = mountRef.current.clientWidth;
    const height = mountRef.current.clientHeight;
    const camera = new THREE.PerspectiveCamera(50, width / height, 0.1, 10000);
    camera.position.set(0, -150, 150);
    camera.lookAt(0, 0, 0);
    cameraRef.current = camera;

    // Renderer
    const renderer = new THREE.WebGLRenderer({ 
      antialias: true,
      alpha: true 
    });
    renderer.setSize(width, height);
    renderer.setPixelRatio(window.devicePixelRatio);
    mountRef.current.appendChild(renderer.domElement);
    rendererRef.current = renderer;

    // Controls
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.maxDistance = 500;
    controls.minDistance = 10;
    controlsRef.current = controls;

    // Grid helper - XY plane for god's eye view
    const gridHelper = new THREE.GridHelper(200, 20, 0x444444, 0x222222);
    gridHelper.rotation.x = Math.PI / 2;
    scene.add(gridHelper);

    // Axis helper
    const axisHelper = new THREE.AxesHelper(50);
    scene.add(axisHelper);

    // Lighting
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    scene.add(ambientLight);
    
    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.4);
    directionalLight.position.set(100, 100, 50);
    directionalLight.castShadow = true;
    scene.add(directionalLight);

    // Handle resize
    const handleResize = () => {
      if (!mountRef.current) return;
      const width = mountRef.current.clientWidth;
      const height = mountRef.current.clientHeight;
      camera.aspect = width / height;
      camera.updateProjectionMatrix();
      renderer.setSize(width, height);
    };
    window.addEventListener('resize', handleResize);

    // Render loop
    const animate = () => {
      frameRef.current = requestAnimationFrame(animate);
      controls.update();
      renderer.render(scene, camera);
    };
    animate();

    return () => {
      window.removeEventListener('resize', handleResize);
      if (frameRef.current) {
        cancelAnimationFrame(frameRef.current);
      }
      mountRef.current?.removeChild(renderer.domElement);
      controls.dispose();
      renderer.dispose();
      
      // Clean up geometries and materials
      jumperMeshes.current.forEach(mesh => {
        mesh.geometry.dispose();
        (mesh.material as THREE.Material).dispose();
      });
      trailLines.current.forEach(line => {
        line.geometry.dispose();
        (line.material as THREE.Material).dispose();
      });
    };
  }, []);

  // Update jumper positions
  useEffect(() => {
    if (!sceneRef.current) return;

    try {
      const positions = projectFormationAtTime(
        formation.participants,
        state.currentTime,
        state.baseJumperId,
        dzCenter
      );

      // Update jumper spheres
      positions.forEach(pos => {
        let mesh = jumperMeshes.current.get(pos.userId);

        if (!mesh) {
          // Create new jumper mesh
          const geometry = new THREE.SphereGeometry(2, 16, 16);
          const material = new THREE.MeshPhongMaterial({ 
            color: new THREE.Color(pos.color),
            transparent: true,
            opacity: 0.9,
            emissive: new THREE.Color(pos.color),
            emissiveIntensity: 0.2
          });
          mesh = new THREE.Mesh(geometry, material);
          mesh.castShadow = true;
          mesh.receiveShadow = true;
          sceneRef.current.add(mesh);
          jumperMeshes.current.set(pos.userId, mesh);

          // Add name label (using sprite)
          const canvas = document.createElement('canvas');
          const context = canvas.getContext('2d');
          canvas.width = 256;
          canvas.height = 64;
          if (context) {
            context.font = '24px Arial';
            context.fillStyle = pos.color;
            context.textAlign = 'center';
            context.fillText(pos.name, 128, 40);
          }
          const texture = new THREE.CanvasTexture(canvas);
          const spriteMaterial = new THREE.SpriteMaterial({ 
            map: texture,
            transparent: true 
          });
          const sprite = new THREE.Sprite(spriteMaterial);
          sprite.scale.set(20, 5, 1);
          sprite.position.y = 5;
          mesh.add(sprite);
        }

        // Update position based on view mode
        const displayPos = state.viewMode === 'godsEye' 
          ? { x: pos.position.x, y: pos.position.y, z: 0 }
          : { x: pos.position.x, y: -pos.position.z, z: pos.position.y };

        mesh.position.set(displayPos.x, displayPos.y, displayPos.z);

        // Update opacity for data gaps
        const material = mesh.material as THREE.MeshPhongMaterial;
        material.opacity = pos.isDataGap ? 0.5 : 0.9;
      });

      // Update camera view for view mode changes
      if (state.viewMode === 'side' && controlsRef.current) {
        controlsRef.current.target.set(0, 0, 0);
        cameraRef.current?.position.set(0, 0, 150);
      }

    } catch (error) {
      console.error('Error projecting formation:', error);
    }
  }, [formation, state.currentTime, state.viewMode, state.baseJumperId, dzCenter]);

  // Update trails
  useEffect(() => {
    if (!sceneRef.current || !state.showTrails) {
      // Remove all trails
      trailLines.current.forEach(line => {
        sceneRef.current?.remove(line);
      });
      return;
    }

    // Calculate trail time range
    const trailStart = Math.max(0, state.currentTime - state.trailLength);
    const trailEnd = state.currentTime;
    const trailSteps = Math.ceil((trailEnd - trailStart) * 4); // 4Hz data

    formation.participants.forEach(participant => {
      if (!participant.isVisible) return;

      const points: THREE.Vector3[] = [];
      
      // Sample trail points
      for (let i = 0; i <= trailSteps; i++) {
        const t = trailStart + (i / trailSteps) * (trailEnd - trailStart);
        try {
          const projected = projectFormationAtTime(
            formation.participants,
            t,
            state.baseJumperId,
            dzCenter
          );
          
          const pos = projected.find(p => p.userId === participant.userId);
          if (pos) {
            const point = state.viewMode === 'godsEye'
              ? new THREE.Vector3(pos.position.x, pos.position.y, 0)
              : new THREE.Vector3(pos.position.x, -pos.position.z, pos.position.y);
            points.push(point);
          }
        } catch (e) {
          // Skip this point
        }
      }

      if (points.length > 1) {
        let line = trailLines.current.get(participant.userId);
        
        if (!line) {
          const geometry = new THREE.BufferGeometry();
          const material = new THREE.LineBasicMaterial({
            color: new THREE.Color(participant.color),
            opacity: 0.5,
            transparent: true
          });
          line = new THREE.Line(geometry, material);
          sceneRef.current.add(line);
          trailLines.current.set(participant.userId, line);
        }

        // Update trail geometry
        line.geometry.setFromPoints(points);
      }
    });
  }, [formation, state.currentTime, state.showTrails, state.trailLength, 
      state.baseJumperId, state.viewMode, dzCenter]);

  // Playback animation
  useEffect(() => {
    if (!state.isPlaying) return;

    let lastTimestamp: number | null = null;
    let animationId: number;

    const animate = (timestamp: number) => {
      if (!lastTimestamp) lastTimestamp = timestamp;
      const deltaTime = (timestamp - lastTimestamp) / 1000;

      setState(prev => {
        const newTime = prev.currentTime + (deltaTime * prev.playbackSpeed);
        if (newTime >= getMaxTime()) {
          return { ...prev, currentTime: getMaxTime(), isPlaying: false };
        }
        return { ...prev, currentTime: newTime };
      });

      lastTimestamp = timestamp;
      animationId = requestAnimationFrame(animate);
    };

    animationId = requestAnimationFrame(animate);
    return () => cancelAnimationFrame(animationId);
  }, [state.isPlaying, state.playbackSpeed, getMaxTime]);

  // Playback controls
  const togglePlayback = () => {
    setState(prev => ({ ...prev, isPlaying: !prev.isPlaying }));
  };

  const handleTimeChange = (value: number) => {
    setState(prev => ({ ...prev, currentTime: value, isPlaying: false }));
  };

  const handleSpeedChange = (value: string | null) => {
    if (value) {
      setState(prev => ({ ...prev, playbackSpeed: parseFloat(value) }));
    }
  };

  const handleViewModeChange = (value: string | null) => {
    if (value === 'godsEye' || value === 'side') {
      setState(prev => ({ ...prev, viewMode: value }));
    }
  };

  const handleBaseChange = (value: string | null) => {
    if (value) {
      setState(prev => ({ ...prev, baseJumperId: value }));
      onBaseChange?.(value);
    }
  };

  return (
    <Stack spacing="md" style={{ width: '100%', height: '100%' }}>
      {/* 3D Viewport */}
      <div 
        ref={mountRef} 
        style={{ 
          width: '100%', 
          height: '600px',
          position: 'relative',
          border: '1px solid #444',
          borderRadius: '8px',
          overflow: 'hidden'
        }} 
      />

      {/* Playback Controls */}
      <Group position="apart">
        <Group>
          <Button
            onClick={togglePlayback}
            leftIcon={state.isPlaying ? <IconPlayerPause /> : <IconPlayerPlay />}
            variant="filled"
          >
            {state.isPlaying ? 'Pause' : 'Play'}
          </Button>
          
          <Select
            value={state.playbackSpeed.toString()}
            onChange={handleSpeedChange}
            data={[
              { value: '0.25', label: '0.25x' },
              { value: '0.5', label: '0.5x' },
              { value: '1', label: '1x' },
              { value: '2', label: '2x' },
              { value: '4', label: '4x' }
            ]}
            style={{ width: '100px' }}
          />

          <Select
            value={state.viewMode}
            onChange={handleViewModeChange}
            data={[
              { value: 'godsEye', label: "God's Eye View" },
              { value: 'side', label: 'Side View' }
            ]}
            style={{ width: '150px' }}
          />
        </Group>

        <Group>
          <Text size="sm">Base:</Text>
          <Select
            value={state.baseJumperId}
            onChange={handleBaseChange}
            data={formation.participants.map(p => ({
              value: p.userId,
              label: p.name
            }))}
            style={{ width: '200px' }}
          />
        </Group>
      </Group>

      {/* Time Slider */}
      <Group style={{ width: '100%' }}>
        <Text size="sm" style={{ minWidth: '60px' }}>
          {state.currentTime.toFixed(1)}s
        </Text>
        <Slider
          value={state.currentTime}
          onChange={handleTimeChange}
          min={0}
          max={getMaxTime()}
          step={0.1}
          style={{ flex: 1 }}
          label={(value) => `${value.toFixed(1)}s`}
        />
        <Text size="sm" style={{ minWidth: '60px' }}>
          {getMaxTime().toFixed(1)}s
        </Text>
      </Group>

      {/* Trail Controls */}
      <Group>
        <Button
          variant={state.showTrails ? 'filled' : 'outline'}
          onClick={() => setState(prev => ({ ...prev, showTrails: !prev.showTrails }))}
        >
          Trails
        </Button>
        {state.showTrails && (
          <>
            <Text size="sm">Length:</Text>
            <Slider
              value={state.trailLength}
              onChange={(value) => setState(prev => ({ ...prev, trailLength: value }))}
              min={1}
              max={10}
              step={0.5}
              style={{ width: '200px' }}
              label={(value) => `${value}s`}
            />
          </>
        )}
      </Group>
    </Stack>
  );
};