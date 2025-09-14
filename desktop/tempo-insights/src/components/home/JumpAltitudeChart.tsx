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

interface ChartDataPoint {
  time: number; // seconds from start
  altitude: number; // feet
  event?: 'exit' | 'deploy' | 'landing';
}

interface JumpAltitudeChartProps {
  jumpId: string;
  exitTime?: number; // seconds from start
  deployTime?: number;
  landingTime?: number;
  exitAltitude?: number;
  deployAltitude?: number;
}

// Generate mock altitude data for visualization
function generateMockAltitudeData(
  exitTime: number = 20,
  deployTime: number = 75,
  landingTime: number = 180,
  exitAlt: number = 14000,
  deployAlt: number = 3500
): ChartDataPoint[] {
  const data: ChartDataPoint[] = [];
  const sampleRate = 2; // Data points per second

  for (let t = 0; t <= landingTime + 10; t += 1 / sampleRate) {
    let altitude: number;
    let event: 'exit' | 'deploy' | 'landing' | undefined;

    if (t < exitTime - 10) {
      // Climb phase
      altitude = 3000 + (t / (exitTime - 10)) * (exitAlt - 3000);
    } else if (t < exitTime) {
      // Level flight
      altitude = exitAlt;
      if (Math.abs(t - exitTime) < 0.1) event = 'exit';
    } else if (t < deployTime) {
      // Freefall
      const fallTime = t - exitTime;
      const totalFallTime = deployTime - exitTime;
      // Use a more realistic fall profile (accelerating then terminal velocity)
      const normalizedTime = fallTime / totalFallTime;
      const altitudeLost = (exitAlt - deployAlt) * normalizedTime;
      altitude = exitAlt - altitudeLost;
      if (Math.abs(t - deployTime) < 0.1) event = 'deploy';
    } else if (t < landingTime) {
      // Under canopy
      const canopyTime = t - deployTime;
      const totalCanopyTime = landingTime - deployTime;
      altitude = deployAlt - (deployAlt * canopyTime / totalCanopyTime);
      if (Math.abs(t - landingTime) < 0.1) event = 'landing';
    } else {
      // On ground
      altitude = 0;
    }

    data.push({
      time: Math.round(t * 10) / 10,
      altitude: Math.round(altitude),
      event,
    });
  }

  return data;
}

export function JumpAltitudeChart({
  jumpId,
  exitTime = 20,
  deployTime = 75,
  landingTime = 180,
  exitAltitude = 14000,
  deployAltitude = 3500,
}: JumpAltitudeChartProps) {
  // Generate chart data
  const chartData = useMemo(
    () => generateMockAltitudeData(exitTime, deployTime, landingTime, exitAltitude, deployAltitude),
    [exitTime, deployTime, landingTime, exitAltitude, deployAltitude]
  );

  // Find event points
  const exitPoint = chartData.find(d => d.event === 'exit');
  const deployPoint = chartData.find(d => d.event === 'deploy');
  const landingPoint = chartData.find(d => d.event === 'landing');

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
          fill={colors[payload.event as 'exit' | 'deploy' | 'landing']}
          stroke="#ffffff"
          strokeWidth={2}
        />
      );
    }
    // Always return a valid SVG element (invisible dot for non-events)
    return (
      <circle
        cx={cx}
        cy={cy}
        r={0}
        fill="none"
        stroke="none"
      />
    );
  };

  // Custom tooltip
  const CustomTooltip = ({ active, payload, label }: any) => {
    if (active && payload && payload[0]) {
      const data = payload[0].payload;
      return (
        <Card p="xs" withBorder>
          <Text size="sm" fw={500}>
            {Math.round(label)}s
          </Text>
          <Text size="xs" c="dimmed">
            {data.altitude.toLocaleString()} ft
          </Text>
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

  return (
    <Card withBorder p="md">
      <Group justify="space-between" mb="md">
        <Text fw={500}>Altitude Profile</Text>
        <Group gap="xs">
          <Badge size="xs" color="green">Exit</Badge>
          <Badge size="xs" color="orange">Deploy</Badge>
          <Badge size="xs" color="red">Landing</Badge>
        </Group>
      </Group>

      <ResponsiveContainer width="100%" height={300}>
        <LineChart
          data={chartData}
          margin={{ top: 5, right: 20, left: 10, bottom: 40 }}
        >
          <CartesianGrid strokeDasharray="3 3" stroke="#004455" />
          <XAxis
            dataKey="time"
            stroke="#ffffff"
            label={{
              value: 'Time (seconds)',
              position: 'insideBottom',
              offset: -10,
              style: { fill: '#ffffff' },
            }}
          />
          <YAxis
            stroke="#ffffff"
            label={{
              value: 'Altitude (ft)',
              angle: -90,
              position: 'insideLeft',
              style: { fill: '#ffffff' },
            }}
            tickFormatter={(value) => `${(value / 1000).toFixed(0)}k`}
          />
          <Tooltip content={<CustomTooltip />} />
          
          {/* Event markers */}
          {exitPoint && (
            <ReferenceLine
              x={exitPoint.time}
              stroke="#00ff88"
              strokeDasharray="3 3"
              opacity={0.5}
            />
          )}
          {deployPoint && (
            <ReferenceLine
              x={deployPoint.time}
              stroke="#ffaa00"
              strokeDasharray="3 3"
              opacity={0.5}
            />
          )}
          {landingPoint && (
            <ReferenceLine
              x={landingPoint.time}
              stroke="#ff3355"
              strokeDasharray="3 3"
              opacity={0.5}
            />
          )}
          
          <Line
            type="monotone"
            dataKey="altitude"
            stroke="#66ccff"
            strokeWidth={2}
            dot={renderEventDot}
          />
        </LineChart>
      </ResponsiveContainer>
    </Card>
  );
}