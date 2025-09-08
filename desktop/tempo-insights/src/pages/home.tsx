import { Container, Title, Grid, Paper, Text, Stack, Group, Badge, Button, Card, Table } from '@mantine/core';
import { AppLayout } from '../components/AppLayout';
import { AuthGuard } from '../components/AuthGuard';
import { useEffect, useState } from 'react';
import { notifications } from '@mantine/notifications';
import { IconCheck, IconX, IconParachute } from '@tabler/icons-react';
import { useRouter } from 'next/router';

interface Invitation {
  id: string;
  code: string;
  group: {
    id?: string;
    name?: string;
    slug?: string;
  };
  groupRole: string;
  invitedBy: string;
  expiresAt: string;
  createdAt: string;
}

interface JumpLog {
  id: string;
  hash: string;
  device: {
    name: string;
    bluetoothId: string;
  };
  createdAt: string;
  flags: any;
  visibleToConnections: boolean;
  exitTime: string | null;
  deploymentAltitude: number | null;
  freefallTime: number | null;
  averageFallRate: number | null;
}

export default function HomePage() {
  const router = useRouter();
  const [invitations, setInvitations] = useState<Invitation[]>([]);
  const [loadingInvitations, setLoadingInvitations] = useState(true);
  const [processingInvite, setProcessingInvite] = useState<string | null>(null);
  const [recentJumps, setRecentJumps] = useState<JumpLog[]>([]);
  const [loadingJumps, setLoadingJumps] = useState(true);

  useEffect(() => {
    loadInvitations();
    loadRecentJumps();
  }, []);

  const loadInvitations = async () => {
    try {
      const response = await fetch('/api/users/invitations');
      if (response.ok) {
        const data = await response.json();
        setInvitations(data.invitations);
      }
    } catch (error) {
      console.error('Error loading invitations:', error);
    } finally {
      setLoadingInvitations(false);
    }
  };

  const loadRecentJumps = async () => {
    try {
      const response = await fetch('/api/jumps/mine?limit=5');
      if (response.ok) {
        const data = await response.json();
        setRecentJumps(data.jumps);
      }
    } catch (error) {
      console.error('Error loading jumps:', error);
    } finally {
      setLoadingJumps(false);
    }
  };

  const handleInvitation = async (code: string, accept: boolean) => {
    setProcessingInvite(code);

    try {
      const response = await fetch(`/api/invitations/${code}/${accept ? 'accept' : 'decline'}`, {
        method: 'POST',
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.error || `Failed to ${accept ? 'accept' : 'decline'} invitation`);
      }

      notifications.show({
        title: 'Success',
        message: data.message,
        color: 'green',
      });

      setInvitations(prev => prev.filter(inv => inv.code !== code));

      if (accept && data.group?.slug) {
        router.push(`/groups/${data.group.slug}`);
      }
    } catch (error) {
      notifications.show({
        title: 'Error',
        message: error instanceof Error ? error.message : 'An error occurred',
        color: 'red',
      });
    } finally {
      setProcessingInvite(null);
    }
  };

  const handleVisibilityToggle = async (jumpId: string, currentVisibility: boolean) => {
    try {
      const response = await fetch(`/api/jumps/${jumpId}/visibility`, {
        method: 'PATCH',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          visibleToConnections: !currentVisibility,
        }),
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.error || 'Failed to update visibility');
      }

      // Update local state
      setRecentJumps(prev => prev.map(jump =>
        jump.id === jumpId
          ? { ...jump, visibleToConnections: data.visibleToConnections }
          : jump
      ));

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
    }
  };

  return (
    <AuthGuard>
      <AppLayout>
        <Container fluid>
          <Title order={2} mb="xl">Dashboard</Title>

          {/* Pending Invitations - keep existing */}
          {invitations.length > 0 && (
            <Paper p="md" withBorder mb="xl" style={{ backgroundColor: 'var(--mantine-color-dark-8)' }}>
              {/* ... existing invitation code ... */}
            </Paper>
          )}

          <Grid>
            <Grid.Col span={{ base: 12, md: 6 }}>
              <Paper p="md" withBorder h="300px">
                <Group justify="space-between" mb="md">
                  <Group>
                    <IconParachute size={20} />
                    <Title order={4}>Recent Jumps</Title>
                  </Group>
                  {recentJumps.length > 0 && (
                    <Button size="xs" variant="subtle">
                      View All
                    </Button>
                  )}
                </Group>

                {loadingJumps ? (
                  <Text c="dimmed">Loading jumps...</Text>
                ) : recentJumps.length === 0 ? (
                  <Text c="dimmed">No jumps recorded yet</Text>
                ) : (
                  <Stack gap="xs">
                    {recentJumps.map(jump => (
                      <Card key={jump.id} p="xs" withBorder>
                        <Group justify="space-between">
                          <div>
                            <Group gap="xs">
                              <Text size="sm" fw={500}>
                                {new Date(jump.createdAt).toLocaleDateString()}
                              </Text>
                              <Badge
                                size="xs"
                                color={jump.visibleToConnections ? 'green' : 'gray'}
                                variant="dot"
                                style={{ cursor: 'pointer' }}
                                onClick={() => handleVisibilityToggle(jump.id, jump.visibleToConnections)}
                              >
                                {jump.visibleToConnections ? 'Visible' : 'Hidden'}
                              </Badge>
                            </Group>
                            <Text size="xs" c="dimmed">
                              {jump.device.name}
                            </Text>
                          </div>
                          <div style={{ textAlign: 'right' }}>
                            {jump.freefallTime ? (
                              <>
                                <Text size="sm">{jump.freefallTime}s</Text>
                                <Text size="xs" c="dimmed">
                                  {jump.averageFallRate} mph
                                </Text>
                              </>
                            ) : (
                              <Badge size="sm" variant="outline">
                                Pending Analysis
                              </Badge>
                            )}
                          </div>
                        </Group>
                      </Card>
                    ))}
                  </Stack>
                )}
              </Paper>
            </Grid.Col>

            <Grid.Col span={{ base: 12, md: 6 }}>
              <Paper p="md" withBorder h="300px">
                <Title order={4} mb="md">My Devices</Title>
                <Text c="dimmed">No devices registered</Text>
              </Paper>
            </Grid.Col>

            <Grid.Col span={12}>
              <Paper p="md" withBorder>
                <Title order={4} mb="md">Formation Skydives</Title>
                <Text c="dimmed">No formation skydives recorded yet</Text>
              </Paper>
            </Grid.Col>
          </Grid>
        </Container>
      </AppLayout>
    </AuthGuard>
  );
}
