// components/home/FormationJumpsPanel.tsx

import React, { useEffect, useState } from 'react';
import { Card, Text, Badge, Group, Stack, Skeleton, ScrollArea, Avatar, Tooltip } from '@mantine/core';
import { IconUsers, IconEye, IconEyeOff } from '@tabler/icons-react';

interface Participant {
  position: number;
  userId: string;
  userName: string;
  userSlug: string;
  jumpLogId: string;
  exitTime: string | null;
  exitAltitude: number | null;
  freefallTime: number | null;
  jumpVisible: boolean;
}

interface Formation {
  id: string;
  name: string;
  jumpTime: string;
  altitude: number | null;
  notes: string | null;
  isPublic: boolean;
  createdAt: string;
  participantCount: number;
  participants: Participant[];
  myPosition: number | null;
}

interface FormationJumpsPanelProps {
  onFormationSelect?: (formationId: string) => void;
}

export function FormationJumpsPanel({ onFormationSelect }: FormationJumpsPanelProps) {
  const [formations, setFormations] = useState<Formation[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    fetchFormations();
    
    // Poll for updates every 30 seconds
    const interval = setInterval(fetchFormations, 30000);
    return () => clearInterval(interval);
  }, []);

  const fetchFormations = async () => {
    try {
      const response = await fetch('/api/formations/mine?limit=5');
      if (!response.ok) {
        throw new Error('Failed to fetch formations');
      }
      
      const data = await response.json();
      setFormations(data.formations);
      setError(null);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to load formations');
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

  const getParticipantInitials = (name: string) => {
    const parts = name.split(' ');
    if (parts.length >= 2) {
      return `${parts[0][0]}${parts[1][0]}`.toUpperCase();
    }
    return name.substring(0, 2).toUpperCase();
  };

  if (loading) {
    return (
      <Card>
        <Text size="lg" fw={600} mb="md">Formation Jumps</Text>
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
        <Text size="lg" fw={600} mb="md">Formation Jumps</Text>
        <Text color="red" size="sm">{error}</Text>
      </Card>
    );
  }

  return (
    <Card>
      <Text size="lg" fw={600} mb="md">Formation Jumps</Text>
      
      <ScrollArea style={{ height: 400 }}>
        <Stack gap="sm">
          {formations.length === 0 ? (
            <Text color="dimmed" size="sm">No formation jumps yet</Text>
          ) : (
            formations.map(formation => (
              <Card
                key={formation.id}
                p="sm"
                withBorder
                style={{ cursor: 'pointer' }}
                onClick={() => onFormationSelect?.(formation.id)}
              >
                <Group justify="space-between" mb="xs">
                  <Group gap="xs">
                    <IconUsers size={16} />
                    <Text size="sm" fw={500}>
                      {formation.name}
                    </Text>
                  </Group>
                  <Badge size="xs" color={formation.isPublic ? 'blue' : 'gray'}>
                    {formation.isPublic ? 'Public' : 'Private'}
                  </Badge>
                </Group>

                <Text size="xs" color="dimmed" mb="xs">
                  {formatDate(formation.jumpTime)}
                  {formation.altitude && ` • ${formation.altitude.toLocaleString()}ft`}
                </Text>

                <Group gap="xs">
                  <Avatar.Group spacing="sm">
                    {formation.participants.slice(0, 4).map((participant, idx) => (
                      <Tooltip
                        key={participant.userId}
                        label={
                          <div>
                            <Text size="xs">{participant.userName}</Text>
                            {participant.freefallTime && participant.jumpVisible && (
                              <Text size="xs" color="dimmed">
                                {Math.round(participant.freefallTime)}s freefall
                              </Text>
                            )}
                          </div>
                        }
                      >
                        <Avatar
                          size="sm"
                          radius="xl"
                          color={participant.userId === formation.participants[0].userId ? 'blue' : 'gray'}
                        >
                          {getParticipantInitials(participant.userName)}
                        </Avatar>
                      </Tooltip>
                    ))}
                    {formation.participantCount > 4 && (
                      <Avatar size="sm" radius="xl">
                        +{formation.participantCount - 4}
                      </Avatar>
                    )}
                  </Avatar.Group>

                  {formation.myPosition && (
                    <Badge size="xs" variant="filled" color="blue">
                      Position {formation.myPosition}
                    </Badge>
                  )}
                </Group>

                {formation.participants.some(p => !p.jumpVisible) && (
                  <Group gap={4} mt="xs">
                    <IconEyeOff size={14} color="gray" />
                    <Text size="xs" color="dimmed">
                      Some jumps are private
                    </Text>
                  </Group>
                )}
              </Card>
            ))
          )}
        </Stack>
      </ScrollArea>
    </Card>
  );
}