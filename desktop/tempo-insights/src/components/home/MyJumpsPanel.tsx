// components/home/MyJumpsPanel.tsx

import React, { useEffect, useState } from 'react';
import { Card, Text, Badge, Group, Stack, Skeleton, ScrollArea, Menu, ActionIcon } from '@mantine/core';
import { IconClock, IconRuler, IconAlertCircle, IconDots, IconUpload } from '@tabler/icons-react';

interface Jump {
  id: string;
  createdAt: string;
  deviceName: string;
  exitTimestamp: string | null;
  exitAltitude: number | null;
  deployAltitude: number | null;
  freefallTime: number | null;
  avgFallRate: number | null;
  analyzed: boolean;
  hasIssues: boolean;
  message: string | null;
  hasGPS: boolean;
  visible: boolean;
}

interface MyJumpsPanelProps {
  onJumpSelect?: (jumpId: string) => void;
  onImportClick?: () => void;
}

export function MyJumpsPanel({ onJumpSelect, onImportClick }: MyJumpsPanelProps) {
  const [jumps, setJumps] = useState<Jump[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    fetchJumps();
    
    // Poll for updates every 30 seconds
    const interval = setInterval(fetchJumps, 30000);
    return () => clearInterval(interval);
  }, []);

  const fetchJumps = async () => {
    try {
      const response = await fetch('/api/jumps/mine?limit=5');
      if (!response.ok) {
        throw new Error('Failed to fetch jumps');
      }
      
      const data = await response.json();
      setJumps(data.jumps);
      setError(null);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to load jumps');
    } finally {
      setLoading(false);
    }
  };

  const formatDate = (dateString: string) => {
    const date = new Date(dateString);
    const today = new Date();
    const yesterday = new Date(today);
    yesterday.setDate(yesterday.getDate() - 1);
    
    if (date.toDateString() === today.toDateString()) {
      return `Today ${date.toLocaleTimeString('en-US', { hour: 'numeric', minute: '2-digit' })}`;
    } else if (date.toDateString() === yesterday.toDateString()) {
      return `Yesterday ${date.toLocaleTimeString('en-US', { hour: 'numeric', minute: '2-digit' })}`;
    } else {
      return date.toLocaleDateString('en-US', { month: 'short', day: 'numeric' });
    }
  };

  const formatFreefallTime = (seconds: number | null) => {
    if (seconds === null) return 'N/A';
    return `${Math.round(seconds)}s`;
  };

  const formatFallRate = (mph: number | null) => {
    if (mph === null) return 'N/A';
    return `${Math.round(mph)} mph`;
  };

  if (loading) {
    return (
      <Card>
        <Text size="lg" fw={600} mb="md">My Recent Jumps</Text>
        <Stack gap="sm">
          {[1, 2, 3].map(i => (
            <Skeleton key={i} height={80} radius="sm" />
          ))}
        </Stack>
      </Card>
    );
  }

  if (error) {
    return (
      <Card>
        <Text size="lg" fw={600} mb="md">My Recent Jumps</Text>
        <Text color="red" size="sm">{error}</Text>
      </Card>
    );
  }

  return (
    <Card>
      <Group justify="space-between" mb="md">
        <Text size="lg" fw={600}>My Recent Jumps</Text>
        <Menu withinPortal position="bottom-end" shadow="sm">
          <Menu.Target>
            <ActionIcon variant="subtle" color="gray">
              <IconDots size={20} />
            </ActionIcon>
          </Menu.Target>

          <Menu.Dropdown>
            <Menu.Item
              leftSection={<IconUpload size={16} />}
              onClick={onImportClick}
            >
              Import...
            </Menu.Item>
          </Menu.Dropdown>
        </Menu>
      </Group>
      
      <ScrollArea style={{ height: 400 }}>
        <Stack gap="sm">
          {jumps.length === 0 ? (
            <Text color="dimmed" size="sm">No jumps recorded yet</Text>
          ) : (
            jumps.map(jump => (
              <Card
                key={jump.id}
                p="sm"
                withBorder
                style={{ cursor: 'pointer' }}
                onClick={() => onJumpSelect?.(jump.id)}
              >
                <Group justify="space-between" mb="xs">
                  <Text size="sm" fw={500}>
                    {formatDate(jump.exitTimestamp || jump.createdAt)}
                  </Text>
                  <Badge size="xs" color={jump.visible ? 'green' : 'gray'}>
                    {jump.visible ? 'Visible' : 'Private'}
                  </Badge>
                </Group>

                <Group gap="xs" mb="xs">
                  <Text size="xs" color="dimmed">{jump.deviceName}</Text>
                  {jump.hasGPS && (
                    <Badge size="xs" variant="dot" color="blue">GPS</Badge>
                  )}
                </Group>

                {jump.analyzed ? (
                  <>
                    {jump.hasIssues ? (
                      <Group gap="xs">
                        <IconAlertCircle size={16} color="orange" />
                        <Text size="sm" color="orange">{jump.message}</Text>
                      </Group>
                    ) : (
                      <Group gap="md">
                        <Group gap={4}>
                          <IconClock size={16} />
                          <Text size="sm">{formatFreefallTime(jump.freefallTime)}</Text>
                        </Group>
                        <Group gap={4}>
                          <IconRuler size={16} />
                          <Text size="sm">{formatFallRate(jump.avgFallRate)}</Text>
                        </Group>
                      </Group>
                    )}
                  </>
                ) : (
                  <Badge size="sm" color="gray">Processing...</Badge>
                )}
              </Card>
            ))
          )}
        </Stack>
      </ScrollArea>
    </Card>
  );
}