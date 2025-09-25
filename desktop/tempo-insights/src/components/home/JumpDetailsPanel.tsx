// components/home/JumpDetailsPanel.tsx

import React, { useEffect, useState } from 'react';
import { Card, Text, Group, Stack, Badge, Divider, Skeleton, SimpleGrid, Paper, Title, Switch, Textarea, Button } from '@mantine/core';
import { IconClock, IconRuler, IconMapPin, IconParachute, IconCalendar, IconEye, IconEyeOff, IconMessage2Up } from '@tabler/icons-react';
import { JumpAltitudeChart } from './JumpAltitudeChart';
import { VelocityBinChart } from './VelocityBinChart';
import { notifications } from '@mantine/notifications';
import type { TimeSeriesPoint } from '../../lib/analysis/log-parser';

interface JumpTimeSeries {
  altitude: TimeSeriesPoint[];
  vspeed: TimeSeriesPoint[];
  gps: any[];
  duration: number;
  sampleRate: number;
  hasGPS: boolean;
  exitOffsetSec?: number;
  deploymentOffsetSec?: number;
  landingOffsetSec?: number;
}

interface VelocityBinData {
  fallRate_mph: number;
  elapsed_sec: number;
}

interface VelocityBinResponse {
  velocityBins: VelocityBinData[];
  summary: {
    totalAnalysisTime: number;
    averageFallRate: number;
    minFallRate: number | null;
    maxFallRate: number | null;
    analysisWindow: {
      startOffset: number;
      endOffset: number;
      duration: number;
    };
  };
}

interface JumpDetails {
  id: string;
  hash: string;
  jumpNumber: number | null;
  device: {
    name: string;
    bluetoothId: string;
  };
  user: {
    id: string;
    name: string;
    slug: string;
  };
  createdAt: string;
  updatedAt: string;
  flags: any;
  visibleToConnections: boolean;
  isOwner: boolean;
  
  // Analysis data from API
  exitTimestamp: string | null;
  exitAltitude: number | null;
  deploymentAltitude: number | null;
  landingTimestamp: string | null;
  freefallTime: number | null;
  averageFallRate: number | null;
  maxSpeed: number | null;
  notes: string | null;
  
  // Time series data
  timeSeries: JumpTimeSeries | null;
}

interface JumpDetailsPanelProps {
  jumpId: string | null;
}

export function JumpDetailsPanel({ jumpId }: JumpDetailsPanelProps) {
  const [jump, setJump] = useState<JumpDetails | null>(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [updatingVisibility, setUpdatingVisibility] = useState(false);
  const [editingNotes, setEditingNotes] = useState(false);
  const [notesValue, setNotesValue] = useState('');
  const [savingNotes, setSavingNotes] = useState(false);
  
  // Velocity bin data
  const [velocityBinData, setVelocityBinData] = useState<VelocityBinResponse | null>(null);
  const [loadingVelocityBins, setLoadingVelocityBins] = useState(false);

  useEffect(() => {
    if (jumpId) {
      fetchJumpDetails(jumpId);
    } else {
      setJump(null);
      setVelocityBinData(null);
    }
  }, [jumpId]);

  const fetchJumpDetails = async (id: string) => {
    setLoading(true);
    setError(null);
    
    try {
      const response = await fetch(`/api/jumps/${id}`);
      if (!response.ok) {
        throw new Error('Failed to fetch jump details');
      }
      
      const data = await response.json();
      setJump(data.jump);
      setNotesValue(data.jump.notes || '');
      
      // Fetch velocity bins if analysis is complete
      if (data.jump.exitTimestamp && data.jump.freefallTime) {
        fetchVelocityBins(id);
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to load jump details');
      setJump(null);
    } finally {
      setLoading(false);
    }
  };

  const fetchVelocityBins = async (id: string) => {
    setLoadingVelocityBins(true);
    
    try {
      const response = await fetch(`/api/jumps/${id}/velocity-bins`);
      if (!response.ok) {
        if (response.status === 400) {
          // Analysis incomplete or missing data - this is expected for some jumps
          console.log('Velocity bin analysis not available for this jump');
          return;
        }
        throw new Error('Failed to fetch velocity bins');
      }
      
      const data = await response.json();
      setVelocityBinData(data);
    } catch (err) {
      console.error('Error fetching velocity bins:', err);
      // Don't show error to user as this is optional data
    } finally {
      setLoadingVelocityBins(false);
    }
  };

  const handleVisibilityToggle = async () => {
    if (!jump || !jump.isOwner || updatingVisibility) return;
    
    setUpdatingVisibility(true);
    const newVisibility = !jump.visibleToConnections;
    
    try {
      const response = await fetch(`/api/jumps/${jump.id}/visibility`, {
        method: 'PATCH',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ visibleToConnections: newVisibility }),
      });
      
      if (!response.ok) {
        throw new Error('Failed to update visibility');
      }
      
      const data = await response.json();
      
      // Update local state
      setJump({ ...jump, visibleToConnections: newVisibility });
      
      notifications.show({
        message: data.message,
        color: 'green',
      });
    } catch (error) {
      notifications.show({
        title: 'Error',
        message: error instanceof Error ? error.message : 'Failed to update visibility',
        color: 'red',
      });
    } finally {
      setUpdatingVisibility(false);
    }
  };

  const formatDate = (dateString: string) => {
    const date = new Date(dateString);
    return date.toLocaleString('en-US', {
      month: 'short',
      day: 'numeric',
      year: 'numeric',
      hour: 'numeric',
      minute: '2-digit',
      hour12: true,
    });
  };

  const formatTime = (seconds: number | null) => {
    if (seconds === null) return 'N/A';
    const minutes = Math.floor(seconds / 60);
    const secs = Math.round(seconds % 60);
    return minutes > 0 ? `${minutes}m ${secs}s` : `${secs}s`;
  };

  const handleEditNotes = () => {
    setEditingNotes(true);
    setNotesValue(jump?.notes || '');
  };

  const handleCancelNotes = () => {
    setEditingNotes(false);
    setNotesValue('');
  };

  const handleSaveNotes = async () => {
    if (!jump || savingNotes) return;
    
    setSavingNotes(true);
    
    try {
      const response = await fetch(`/api/jumps/${jump.id}/notes`, {
        method: 'PATCH',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ notes: notesValue || null }),
      });
      
      if (!response.ok) {
        throw new Error('Failed to update notes');
      }
      
      const data = await response.json();
      
      // Update local state
      setJump({ ...jump, notes: notesValue || null });
      setEditingNotes(false);
      
      notifications.show({
        message: data.message,
        color: 'green',
      });
    } catch (error) {
      notifications.show({
        title: 'Error',
        message: error instanceof Error ? error.message : 'Failed to update notes',
        color: 'red',
      });
    } finally {
      setSavingNotes(false);
    }
  };

  if (!jumpId) {
    return (
      <Paper p="md" style={{ textAlign: 'center' }}>
        <IconParachute size={48} stroke={1} style={{ opacity: 0.5 }} />
        <Text mt="md" c="dimmed">Select a jump from the right panel to view details</Text>
      </Paper>
    );
  }

  if (loading) {
    return (
      <Stack>
        <Skeleton height={30} width="60%" />
        <Skeleton height={20} width="40%" />
        <Divider my="md" />
        <SimpleGrid cols={2} spacing="md">
          <Skeleton height={80} />
          <Skeleton height={80} />
          <Skeleton height={80} />
          <Skeleton height={80} />
        </SimpleGrid>
      </Stack>
    );
  }

  if (error || !jump) {
    return (
      <Paper p="md" style={{ textAlign: 'center' }}>
        <Text c="red">{error || 'Jump not found'}</Text>
      </Paper>
    );
  }

  return (
    <Stack>
      {/* Header */}
      <div>
        <Group justify="space-between" align="flex-start">
          <div>
            <Title order={3}>
              Jump Summary
              {jump.jumpNumber && (
                <Text component="span" size="lg" c="dimmed" ml="xs">
                  #{jump.jumpNumber}
                </Text>
              )}
            </Title>
            <Group gap="xs" mt="xs">
              <IconCalendar size={16} />
              <Text size="sm" c="dimmed">{formatDate(jump.exitTimestamp)}</Text>
            </Group>
          </div>
          <Stack gap="xs" align="flex-end">
            {jump.isOwner && (
              <Switch
                checked={jump.visibleToConnections}
                onChange={handleVisibilityToggle}
                disabled={updatingVisibility}
                label={
                  <Group gap={4}>
                    {jump.visibleToConnections ? (
                      <IconEye size={16} />
                    ) : (
                      <IconEyeOff size={16} />
                    )}
                    <Text size="sm">
                      {jump.visibleToConnections ? 'Visible' : 'Private'}
                    </Text>
                  </Group>
                }
                color="green"
              />
            )}
            {!jump.isOwner && (
              <Badge color={jump.visibleToConnections ? 'green' : 'gray'}>
                {jump.visibleToConnections ? 'Visible' : 'Private'}
              </Badge>
            )}
            <Text size="xs" c="dimmed">{jump.device.name}</Text>
          </Stack>
        </Group>
      </div>

      <Divider />

      {/* Analysis Status */}
      {!jump.exitTimestamp && !jump.freefallTime ? (
        <Card withBorder p="lg" style={{ textAlign: 'center' }}>
          <Badge size="lg" color="blue" variant="light">Processing...</Badge>
          <Text size="sm" c="dimmed" mt="sm">Analysis pending</Text>
        </Card>
      ) : (
        <>
          {/* Main Metrics */}
          <SimpleGrid cols={2} spacing="md">
            <Card withBorder p="md">
              <Group gap="sm">
                <IconClock size={24} style={{ opacity: 0.7 }} />
                <div>
                  <Text size="xs" c="dimmed">Exit Time</Text>
                  <Text size="lg" fw={600}>
                    {jump.exitTimestamp 
                      ? new Date(jump.exitTimestamp).toLocaleTimeString('en-US', {
                          hour: 'numeric',
                          minute: '2-digit',
                          second: '2-digit',
                          hour12: true
                        })
                      : 'N/A'
                    }
                  </Text>
                </div>
              </Group>
            </Card>

            <Card withBorder p="md">
              <Group gap="sm">
                <IconParachute size={24} style={{ opacity: 0.7 }} />
                <div>
                  <Text size="xs" c="dimmed">Deploy Altitude</Text>
                  <Text size="lg" fw={600}>
                    {jump.deploymentAltitude 
                      ? `${jump.deploymentAltitude.toLocaleString()} ft`
                      : 'N/A'
                    }
                  </Text>
                </div>
              </Group>
            </Card>

            <Card withBorder p="md">
              <Group gap="sm">
                <IconClock size={24} style={{ opacity: 0.7 }} />
                <div>
                  <Text size="xs" c="dimmed">Freefall Time</Text>
                  <Text size="lg" fw={600}>
                    {formatTime(jump.freefallTime)}
                  </Text>
                </div>
              </Group>
            </Card>

            <Card withBorder p="md">
              <Group gap="sm">
                <IconRuler size={24} style={{ opacity: 0.7 }} />
                <div>
                  <Text size="xs" c="dimmed">Avg Fall Rate</Text>
                  <Text size="lg" fw={600}>
                    {jump.averageFallRate 
                      ? `${Math.round(jump.averageFallRate)} mph`
                      : 'N/A'
                    }
                  </Text>
                </div>
              </Group>
            </Card>
          </SimpleGrid>

          {/* Additional Details */}
          <Card withBorder p="md">
            <Text fw={500} mb="sm">Jump Profile</Text>
            <Stack gap="xs">
              <Group justify="space-between">
                <Text size="sm" c="dimmed">Exit Altitude</Text>
                <Text size="sm">
                  {jump.exitAltitude ? `${jump.exitAltitude.toLocaleString()} ft` : 'N/A'}
                </Text>
              </Group>
              
              {jump.exitAltitude && jump.deploymentAltitude && (
                <Group justify="space-between">
                  <Text size="sm" c="dimmed">Altitude Lost</Text>
                  <Text size="sm">
                    {(jump.exitAltitude - jump.deploymentAltitude).toLocaleString()} ft
                  </Text>
                </Group>
              )}

              {jump.landingTimestamp && jump.exitTimestamp && (
                <Group justify="space-between">
                  <Text size="sm" c="dimmed">Total Jump Time</Text>
                  <Text size="sm">
                    {formatTime((new Date(jump.landingTimestamp).getTime() - new Date(jump.exitTimestamp).getTime()) / 1000)}
                  </Text>
                </Group>
              )}

              {jump.maxSpeed && (
                <Group justify="space-between">
                  <Text size="sm" c="dimmed">Max Speed</Text>
                  <Text size="sm">{Math.round(jump.maxSpeed)} mph</Text>
                </Group>
              )}
            </Stack>
          </Card>

          {/* Altitude Chart with real data */}
          {jump.timeSeries && jump.timeSeries.altitude.length > 0 && (
            <JumpAltitudeChart
              altitudeData={jump.timeSeries.altitude}
              vspeedData={jump.timeSeries.vspeed}
              exitOffsetSec={jump.timeSeries.exitOffsetSec}
              deploymentOffsetSec={jump.timeSeries.deploymentOffsetSec}
              landingOffsetSec={jump.timeSeries.landingOffsetSec}
              showVSpeed={false}
            />
          )}

          {/* Velocity Bin Chart */}
          {loadingVelocityBins && (
            <Card withBorder p="md">
              <Skeleton height={300} />
            </Card>
          )}
          
          {velocityBinData && velocityBinData.velocityBins.length > 0 && (
            <VelocityBinChart
              data={velocityBinData.velocityBins}
              summary={velocityBinData.summary}
            />
          )}
        </>
      )}

      {/* Notes */}
      {(jump.notes || editingNotes || jump.isOwner) && (
        <Card withBorder p="md">
          <Group justify="space-between" mb="sm">
            <Text fw={500}>Notes</Text>
            {jump.isOwner && !editingNotes && (
              <Button
                size="xs"
                variant="subtle"
                onClick={handleEditNotes}
              >
                {jump.notes ? 'Edit' : 'Add Notes'}
              </Button>
            )}
          </Group>
          
          {editingNotes ? (
            <Stack>
              <Textarea
                value={notesValue}
                onChange={(e) => setNotesValue(e.currentTarget.value)}
                placeholder="Add notes about this jump..."
                minRows={3}
                maxRows={10}
                maxLength={5000}
                description={`${notesValue.length}/5000 characters`}
              />
              <Group justify="flex-end" gap="xs">
                <Button
                  size="xs"
                  variant="subtle"
                  onClick={handleCancelNotes}
                  disabled={savingNotes}
                >
                  Cancel
                </Button>
                <Button
                  size="xs"
                  leftSection={<IconMessage2Up size={16} />}
                  onClick={handleSaveNotes}
                  loading={savingNotes}
                >
                  Save
                </Button>
              </Group>
            </Stack>
          ) : (
            <Text size="sm" style={{ whiteSpace: 'pre-wrap' }}>
              {jump.notes || 'No notes added yet.'}
            </Text>
          )}
        </Card>
      )}
    </Stack>
  );
}