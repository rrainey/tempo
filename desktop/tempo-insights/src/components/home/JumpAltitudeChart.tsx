// components/home/JumpAltitudeChart.tsx

import React, { useMemo } from 'react';
import { Card, Text, Badge, Group } from '@mantine/core';
import {
  LineChart,
  Line,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip,
  ReferenceLine,
  ResponsiveContainer,
  Dot,
  Label,
} from 'recharts';
import { TimeSeriesPoint } from '../../lib/analysis/log-parser';

interface ChartDataPoint {
  time: number; // seconds from start
  altitude: number; // feet
  vspeed?: number; // feet per minute
  event?: 'exit' | 'deploy' | 'landing';
}

interface JumpAltitudeChartProps {
  altitudeData: TimeSeriesPoint[];
  vspeedData?: TimeSeriesPoint[];
  exitOffsetSec?: number;
  deploymentOffsetSec?: number;
  landingOffsetSec?: number;
  showVSpeed?: boolean;
}

export function JumpAltitudeChart({
  altitudeData,
  vspeedData,
  exitOffsetSec,
  deploymentOffsetSec,
  landingOffsetSec,
  showVSpeed = false
}: JumpAltitudeChartProps) {
  
  // Prepare chart data
  const chartData = useMemo(() => {
    // Create a map to merge altitude and vspeed data
    const dataMap = new Map<number, ChartDataPoint>();
    
    // Add altitude data
    altitudeData.forEach(point => {
      dataMap.set(point.timestamp, {
        time: point.timestamp,
        altitude: point.value,
        event: undefined
      });
    });
    
    // Add vspeed data if available
    if (vspeedData && showVSpeed) {
      vspeedData.forEach(point => {
        const existing = dataMap.get(point.timestamp);
        if (existing) {
          existing.vspeed = point.value;
        } else {
          dataMap.set(point.timestamp, {
            time: point.timestamp,
            altitude: 0, // Will be interpolated
            vspeed: point.value,
            event: undefined
          });
        }
      });
    }
    
    // Convert to array and sort by time
    const data = Array.from(dataMap.values()).sort((a, b) => a.time - b.time);
    
    // Mark events
    data.forEach(point => {
      if (exitOffsetSec && Math.abs(point.time - exitOffsetSec) < 0.5) {
        point.event = 'exit';
      } else if (deploymentOffsetSec && Math.abs(point.time - deploymentOffsetSec) < 0.5) {
        point.event = 'deploy';
      } else if (landingOffsetSec && Math.abs(point.time - landingOffsetSec) < 0.5) {
        point.event = 'landing';
      }
    });
    
    return data;
  }, [altitudeData, vspeedData, showVSpeed, exitOffsetSec, deploymentOffsetSec, landingOffsetSec]);

  // Custom dot for events
  const renderEventDot = (props: any) => {
    const { cx, cy, payload } = props;
    const colors = {
      exit: '#00ff88',
      deploy: '#ffaa00',
      landing: '#ff3355',
    };

    if (payload.event) {
      return (
        <circle
          cx={cx}
          cy={cy}
          r={6}
          fill={colors[payload.event as keyof typeof colors]}
          stroke="#ffffff"
          strokeWidth={2}
        />
      );
    }
    return null;
  };

  // Custom tooltip
  const CustomTooltip = ({ active, payload, label }: any) => {
    if (active && payload && payload[0]) {
      const data = payload[0].payload;
      return (
        <Card p="xs" withBorder>
          <Text size="sm" fw={500}>
            {label.toFixed(1)}s
          </Text>
          <Text size="xs" c="dimmed">
            {data.altitude.toLocaleString()} ft
          </Text>
          {data.vspeed !== undefined && (
            <Text size="xs" c="dimmed">
              {Math.round(data.vspeed)} fpm
            </Text>
          )}
          {data.event && (
            <Badge size="xs" mt={4} color={
              data.event === 'exit' ? 'green' :
              data.event === 'deploy' ? 'orange' :
              'red'
            }>
              {data.event.toUpperCase()}
            </Badge>
          )}
        </Card>
      );
    }
    return null;
  };

  if (chartData.length === 0) {
    return (
      <Card withBorder p="md">
        <Text c="dimmed" ta="center">No altitude data available</Text>
      </Card>
    );
  }

  // Find min/max for Y axis
  const minAlt = Math.min(...chartData.map(d => d.altitude));
  const maxAlt = Math.max(...chartData.map(d => d.altitude));
  const altRange = maxAlt - minAlt;
  const yMin = Math.max(0, minAlt - altRange * 0.1);
  const yMax = maxAlt + altRange * 0.1;

  return (
    <Card withBorder p="md">
      <Group justify="space-between" mb="md">
        <Text fw={500}>Altitude Profile</Text>
        <Group gap="xs">
          <Badge size="xs" color="green" leftSection={<div style={{
            width: 8, height: 8, borderRadius: '50%', backgroundColor: '#00ff88'
          }} />}>
            Exit
          </Badge>
          <Badge size="xs" color="orange" leftSection={<div style={{
            width: 8, height: 8, borderRadius: '50%', backgroundColor: '#ffaa00'
          }} />}>
            Deploy
          </Badge>
          <Badge size="xs" color="red" leftSection={<div style={{
            width: 8, height: 8, borderRadius: '50%', backgroundColor: '#ff3355'
          }} />}>
            Landing
          </Badge>
        </Group>
      </Group>

      <ResponsiveContainer width="100%" height={300}>
        <LineChart
          data={chartData}
          margin={{ top: 5, right: 20, left: 10, bottom: 40 }}
        >
          <CartesianGrid strokeDasharray="3 3" stroke="#004455" opacity={0.5} />
          <XAxis
            dataKey="time"
            stroke="#c5c0c9"
            label={{
              value: 'Time (seconds)',
              position: 'insideBottom',
              offset: -10,
              style: { fill: '#c5c0c9' },
            }}
            domain={['dataMin', 'dataMax']}
          />
          <YAxis
            stroke="#c5c0c9"
            domain={[yMin, yMax]}
            label={{
              value: 'Altitude (ft)',
              angle: -90,
              position: 'insideLeft',
              style: { fill: '#c5c0c9' },
            }}
            tickFormatter={(value) => value >= 1000 ? `${(value / 1000).toFixed(1)}k` : value}
          />
          <Tooltip content={<CustomTooltip />} />
          
          {/* Event markers */}
          {exitOffsetSec && (
            <ReferenceLine
              x={exitOffsetSec}
              stroke="#00ff88"
              strokeDasharray="5 5"
              opacity={0.7}
            >
              <Label value="Exit" position="top" fill="#00ff88" />
            </ReferenceLine>
          )}
          {deploymentOffsetSec && (
            <ReferenceLine
              x={deploymentOffsetSec}
              stroke="#ffaa00"
              strokeDasharray="5 5"
              opacity={0.7}
            >
              <Label value="Deploy" position="top" fill="#ffaa00" />
            </ReferenceLine>
          )}
          {landingOffsetSec && (
            <ReferenceLine
              x={landingOffsetSec}
              stroke="#ff3355"
              strokeDasharray="5 5"
              opacity={0.7}
            >
              <Label value="Landing" position="bottom" fill="#ff3355" />
            </ReferenceLine>
          )}
          
          <Line
            type="monotone"
            dataKey="altitude"
            stroke="#66ccff"
            strokeWidth={2}
            dot={false}
            activeDot={{ r: 4 }}
          />
          
          {showVSpeed && (
            <Line
              yAxisId="vspeed"
              type="monotone"
              dataKey="vspeed"
              stroke="#855bf0"
              strokeWidth={1}
              strokeDasharray="3 3"
              dot={false}
            />
          )}
          
          {/* Event dots */}
          {chartData.filter(d => d.event).map((point, index) => (
            <Dot
              key={`event-${index}`}
              cx={0}
              cy={0}
              r={6}
              fill={
                point.event === 'exit' ? '#00ff88' :
                point.event === 'deploy' ? '#ffaa00' :
                '#ff3355'
              }
              stroke="#ffffff"
              strokeWidth={2}
            />
          ))}
        </LineChart>
      </ResponsiveContainer>
    </Card>
  );
}