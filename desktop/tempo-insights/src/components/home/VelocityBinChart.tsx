// components/home/VelocityBinChart.tsx

import React, { useState } from 'react';
import { Card, Text, Group, Badge, Stack, Select } from '@mantine/core';
import {
  BarChart,
  Bar,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip,
  ResponsiveContainer,
  Cell,
  Rectangle,
  Legend
} from 'recharts';
import { IconClock, IconRuler } from '@tabler/icons-react';

interface VelocityBinData {
  fallRate_mph: number;
  elapsed_sec: number;
  calibrated_elapsed_sec: number;
}

interface VelocityBinSummary {
  raw: {
    totalAnalysisTime: number;
    averageFallRate: number;
    minFallRate: number | null;
    maxFallRate: number | null;
  };
  calibrated: {
    totalAnalysisTime: number;
    averageFallRate: number;
    minFallRate: number | null;
    maxFallRate: number | null;
  };
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

type DisplayMode = 'raw' | 'calibrated' | 'both';

export function VelocityBinChart({ data, summary }: VelocityBinChartProps) {
  const [displayMode, setDisplayMode] = useState<DisplayMode>('raw');

  // Custom tooltip
  const CustomTooltip = ({ active, payload }: any) => {
    if (active && payload && payload.length > 0) {
      const bin = payload[0].payload;
      
      return (
        <Card p="xs" withBorder>
          <Text size="sm" fw={500}>
            {bin.fallRate_mph} mph
          </Text>
          {displayMode === 'raw' && (
            <>
              <Text size="xs" c="dimmed">
                Raw: {bin.elapsed_sec.toFixed(1)} seconds
              </Text>
              <Text size="xs" c="dimmed">
                {(bin.elapsed_sec / summary.raw.totalAnalysisTime * 100).toFixed(1)}% of time
              </Text>
            </>
          )}
          {displayMode === 'calibrated' && (
            <>
              <Text size="xs" c="dimmed">
                Calibrated: {bin.calibrated_elapsed_sec.toFixed(1)} seconds
              </Text>
              <Text size="xs" c="dimmed">
                {(bin.calibrated_elapsed_sec / summary.calibrated.totalAnalysisTime * 100).toFixed(1)}% of time
              </Text>
            </>
          )}
          {displayMode === 'both' && (
            <>
              <Text size="xs" c="dimmed">
                Raw: {bin.elapsed_sec.toFixed(1)}s ({(bin.elapsed_sec / summary.raw.totalAnalysisTime * 100).toFixed(1)}%)
              </Text>
              <Text size="xs" c="dimmed">
                Cal: {bin.calibrated_elapsed_sec.toFixed(1)}s ({(bin.calibrated_elapsed_sec / summary.calibrated.totalAnalysisTime * 100).toFixed(1)}%)
              </Text>
            </>
          )}
        </Card>
      );
    }
    return null;
  };

  if (!data || data.length === 0) {
    return (
      <Card withBorder p="md">
        <Text c="dimmed" ta="center">No velocity data available</Text>
      </Card>
    );
  }

  // Calculate height based on number of bins
  const chartHeight = Math.max(400, data.length * 20);

  // Get active summary based on display mode
  const activeSummary = displayMode === 'calibrated' ? summary.calibrated : summary.raw;

  return (
    <Card withBorder p="md">
      <Stack>
        {/* Header */}
        <div>
          <Group justify="space-between" align="flex-start">
            <div>
              <Text fw={500}>Fall Rate Distribution</Text>
              <Text size="xs" c="dimmed" mt={4}>
                Time spent at each fall rate after accelerating to terminal velocity
              </Text>
            </div>
            <Group gap="xs">
              <Badge size="sm" variant="light">
                {summary.analysisWindow.startOffset.toFixed(0)}-{summary.analysisWindow.endOffset.toFixed(0)}s
              </Badge>
            </Group>
          </Group>
        </div>

        {/* Display Mode Selector */}
        <Select
          label="Display Mode"
          description="Choose between raw, calibrated (density-corrected), or both"
          value={displayMode}
          onChange={(value) => setDisplayMode(value as DisplayMode)}
          data={[
            { value: 'raw', label: 'Raw Fall Rate' },
            { value: 'calibrated', label: 'Calibrated Fall Rate (Density Corrected)' },
            { value: 'both', label: 'Both (Comparison)' }
          ]}
          allowDeselect={false}
        />

        {/* Summary Stats */}
        <Group grow>
          <Card withBorder p="sm">
            <Group gap="xs">
              <IconRuler size={20} style={{ opacity: 0.7 }} />
              <div>
                <Text size="xs" c="dimmed">
                  {displayMode === 'calibrated' ? 'Avg Calibrated Rate' : 'Avg Raw Rate'}
                </Text>
                <Text fw={600}>{activeSummary.averageFallRate} mph</Text>
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

        {displayMode === 'both' && (
          <Card withBorder p="sm" style={{ backgroundColor: 'rgba(221, 255, 85, 0.05)' }}>
            <Stack gap="xs">
              <Text size="sm" fw={500}>Comparison</Text>
              <Group justify="space-between">
                <Text size="xs" c="dimmed">Raw Avg:</Text>
                <Text size="xs" fw={500}>{summary.raw.averageFallRate} mph</Text>
              </Group>
              <Group justify="space-between">
                <Text size="xs" c="dimmed">Calibrated Avg:</Text>
                <Text size="xs" fw={500}>{summary.calibrated.averageFallRate} mph</Text>
              </Group>
              <Group justify="space-between">
                <Text size="xs" c="dimmed">Difference:</Text>
                <Text size="xs" fw={500} c={summary.calibrated.averageFallRate > summary.raw.averageFallRate ? 'green' : 'blue'}>
                  {Math.abs(summary.calibrated.averageFallRate - summary.raw.averageFallRate)} mph
                  {summary.calibrated.averageFallRate > summary.raw.averageFallRate ? ' faster' : ' slower'}
                </Text>
              </Group>
            </Stack>
          </Card>
        )}

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
            {displayMode === 'both' && <Legend />}
            
            {displayMode === 'raw' && (
              <Bar 
                dataKey="elapsed_sec" 
                fill="#0088ff" 
                radius={[0, 10, 10, 0]} 
                activeBar={<Rectangle fill="#00aaff" stroke="#0088ff" radius={[0, 10, 10, 0]} />}
                name="Raw Fall Rate"
              />
            )}
            
            {displayMode === 'calibrated' && (
              <Bar 
                dataKey="calibrated_elapsed_sec" 
                fill="#ddff55" 
                radius={[0, 10, 10, 0]} 
                activeBar={<Rectangle fill="#eeff88" stroke="#ddff55" radius={[0, 10, 10, 0]} />}
                name="Calibrated Fall Rate"
              />
            )}
            
            {displayMode === 'both' && (
              <>
                <Bar 
                  dataKey="elapsed_sec" 
                  fill="#0088ff" 
                  radius={[0, 10, 10, 0]}
                  name="Raw"
                />
                <Bar 
                  dataKey="calibrated_elapsed_sec" 
                  fill="#ddff55" 
                  radius={[0, 10, 10, 0]}
                  name="Calibrated"
                />
              </>
            )}
          </BarChart>
        </ResponsiveContainer>
      </Stack>
    </Card>
  );
}