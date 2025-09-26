// components/home/VelocityBinChart.tsx

import React from 'react';
import { Card, Text, Group, Badge, Stack } from '@mantine/core';
import {
  BarChart,
  Bar,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip,
  ResponsiveContainer,
  Cell,
  LabelList,
  Rectangle
} from 'recharts';
import { IconClock, IconRuler } from '@tabler/icons-react';

interface VelocityBinData {
  fallRate_mph: number;
  elapsed_sec: number;
}

interface VelocityBinSummary {
  totalAnalysisTime: number;
  averageFallRate: number;
  minFallRate: number | null;
  maxFallRate: number | null;
  analysisWindow: {
    startOffset: number;
    endOffset: number;
    duration: number;
  };
}

interface VelocityBinChartProps {
  data: VelocityBinData[];
  summary: VelocityBinSummary;
}

export function VelocityBinChart({ data, summary }: VelocityBinChartProps) {
  // Custom tooltip
  const CustomTooltip = ({ active, payload }: any) => {
    if (active && payload && payload[0]) {
      const bin = payload[0].payload;
      const percentage = (bin.elapsed_sec / summary.totalAnalysisTime * 100).toFixed(1);
      
      return (
        <Card p="xs" withBorder>
          <Text size="sm" fw={500}>
            {bin.fallRate_mph} mph
          </Text>
          <Text size="xs" c="dimmed">
            {bin.elapsed_sec.toFixed(1)} seconds
          </Text>
          <Text size="xs" c="dimmed">
            {percentage}% of time
          </Text>
        </Card>
      );
    }
    return null;
  };

  // Format time display
  //const formatTime = (seconds: number): string => {
  //  if (seconds < 1) return `${(seconds * 1000).toFixed(0)}ms`;
  //  return `${seconds.toFixed(1)}s`;
  //};

  if (!data || data.length === 0) {
    return (
      <Card withBorder p="md">
        <Text c="dimmed" ta="center">No velocity data available</Text>
      </Card>
    );
  }

  // Calculate height based on number of bins
  const chartHeight = Math.max(400, data.length * 20);

  return (
    <Card withBorder p="md">
      <Stack>
        {/* Header */}
        <div>
          <Group justify="space-between" align="flex-start">
            <Text fw={500}>Fall Rate Distribution</Text>
            <Badge size="sm" variant="light">
              {summary.analysisWindow.startOffset.toFixed(0)}-{summary.analysisWindow.endOffset.toFixed(0)}s
            </Badge>
          </Group>
          <Text size="xs" c="dimmed" mt={4}>
            Time spent at each fall rate after accelerating to terminal velocity
          </Text>
        </div>

        {/* Summary Stats */}
        <Group grow>
          <Card withBorder p="sm">
            <Group gap="xs">
              <IconRuler size={20} style={{ opacity: 0.7 }} />
              <div>
                <Text size="xs" c="dimmed">Average Fall Rate</Text>
                <Text fw={600}>{summary.averageFallRate} mph</Text>
              </div>
            </Group>
          </Card>
          
          <Card withBorder p="sm">
            <Group gap="xs">
              <IconClock size={20} style={{ opacity: 0.7 }} />
              <div>
                <Text size="xs" c="dimmed">Analysis Duration</Text>
                <Text fw={600}>{summary.analysisWindow.duration.toFixed(0)}s</Text>
              </div>
            </Group>
          </Card>
        </Group>

        {/* Chart */}
        <ResponsiveContainer width="100%" height={chartHeight}>
          <BarChart
            data={data}
            layout="vertical"
            margin={{ top: 10, right: 30, left: 40, bottom: 30 }}
          >
            <CartesianGrid 
              strokeDasharray="3 3" 
              stroke="#004455" 
              opacity={0.5}
              horizontal={true}
              vertical={true}
            />
            <XAxis
              type="number"
              domain={[0, 'dataMax']}
              stroke="#c5c0c9"
              label={{
                value: 'Time (seconds)',
                position: 'insideBottom',
                offset: -10,
                style: { fill: '#c5c0c9' },
              }}
            />
            <YAxis
              type="category"
              dataKey="fallRate_mph"
              stroke="#c5c0c9"
              tick={{ fontSize: 12 }}
              label={{
                value: 'Fall Rate (mph)',
                angle: -90,
                position: 'insideLeft',
                style: { fill: '#c5c0c9', textAnchor: 'middle' },
              }}
            />
            <Tooltip 
              content={<CustomTooltip />}
              cursor={{ fill: 'rgba(255, 255, 255, 0.05)' }}
            />
            <Bar dataKey="elapsed_sec" fill="#0088ff" radius={[0,10,10,0]} activeBar={<Rectangle fill="#f2f2f2" stroke="blue" radius={[0,10,10,0]}/>}/>
          </BarChart>
        </ResponsiveContainer>
      </Stack>
    </Card>
  );
}