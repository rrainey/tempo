import { useState, useEffect } from 'react';
import { useRouter } from 'next/router';
import {
  Container,
  Title,
  Paper,
  Text,
  Group,
  Badge,
  Stack,
  Card,
  Loader,
  Center,
  Button,
  Grid,
  ActionIcon
} from '@mantine/core';
import { AppLayout } from '../../components/AppLayout';
import { AuthGuard } from '../../components/AuthGuard';
import { IconParachute, IconEdit } from '@tabler/icons-react';
import Link from 'next/link';
import { EditNotesModal } from '../../components/EditNotesModal';
import ReactMarkdown from 'react-markdown';

interface JumpDetail {
  id: string;
  hash: string;
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
  exitTimestamp: string | null;
  exitAltitude: number | null;
  deploymentAltitude: number | null;
  landingTimestamp: string | null;
  freefallTime: number | null;
  averageFallRate: number | null;
  maxSpeed: number | null;
  notes: string | null;
}

export default function JumpDetailPage() {
  const router = useRouter();
  const { id } = router.query;
  const [jump, setJump] = useState<JumpDetail | null>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [notesModalOpened, setNotesModalOpened] = useState(false);

  useEffect(() => {
    if (id) {
      loadJumpDetails();
    }
  }, [id]);

  const loadJumpDetails = async () => {
    try {
      setLoading(true);
      const response = await fetch(`/api/jumps/${id}`);

      if (!response.ok) {
        if (response.status === 404) {
          setError('Jump not found');
        } else if (response.status === 403) {
          setError('You do not have permission to view this jump');
        } else {
          setError('Failed to load jump details');
        }
        return;
      }

      const data = await response.json();
      setJump(data.jump);
    } catch (err) {
      setError('Failed to load jump details');
    } finally {
      setLoading(false);
    }
  };

  if (loading) return (
    <AuthGuard>
      <AppLayout>
        <Center style={{ height: '50vh' }}>
          <Loader size="lg" color="accent" />
        </Center>
      </AppLayout>
    </AuthGuard>
  );

  if (error || !jump) return (
    <AuthGuard>
      <AppLayout>
        <Container>
          <Text c="red">{error || 'Jump not found'}</Text>
        </Container>
      </AppLayout>
    </AuthGuard>
  );

  return (
    <AuthGuard>
      <AppLayout>
        <Container>
          <Group justify="space-between" mb="xl">
            <Group>
              <IconParachute size={32} />
              <div>
                <Title order={2}>Jump Details</Title>
                <Text size="sm" c="dimmed">
                  {new Date(jump.createdAt).toLocaleDateString()} at{' '}
                  {new Date(jump.createdAt).toLocaleTimeString()}
                </Text>
              </div>
            </Group>
            {jump.isOwner && (
              <Button
                leftSection={<IconEdit size={16} />}
                variant="subtle"
                onClick={() => setNotesModalOpened(true)}
              >
                Edit Notes
              </Button>
            )}
          </Group>

          <Grid>
            <Grid.Col span={{ base: 12, md: 8 }}>
              <Stack gap="md">
                <Paper p="md" withBorder>
                  <Title order={4} mb="md">Jump Information</Title>
                  <Grid>
                    <Grid.Col span={6}>
                      <Text size="sm" c="dimmed">Device</Text>
                      <Text fw={500}>{jump.device.name}</Text>
                    </Grid.Col>
                    <Grid.Col span={6}>
                      <Text size="sm" c="dimmed">Jumper</Text>
                      <Text fw={500}>
                        <Link href={`/users/${jump.user.slug}`}>
                          {jump.user.name}
                        </Link>
                      </Text>
                    </Grid.Col>
                    <Grid.Col span={6}>
                      <Text size="sm" c="dimmed">Visibility</Text>
                      <Badge color={jump.visibleToConnections ? 'green' : 'gray'}>
                        {jump.visibleToConnections ? 'Visible to connections' : 'Private'}
                      </Badge>
                    </Grid.Col>
                    <Grid.Col span={6}>
                      <Text size="sm" c="dimmed">Jump ID</Text>
                      <Text size="xs" style={{ fontFamily: 'monospace' }}>
                        {jump.id}
                      </Text>
                    </Grid.Col>
                  </Grid>
                </Paper>

                <Paper p="md" withBorder>
                  <Title order={4} mb="md">Performance Metrics</Title>
                  {jump.freefallTime ? (
                    <Grid>
                      <Grid.Col span={6}>
                        <Text size="sm" c="dimmed">Exit Altitude</Text>
                        <Text fw={500}>{jump.exitAltitude || 'N/A'} ft</Text>
                      </Grid.Col>
                      <Grid.Col span={6}>
                        <Text size="sm" c="dimmed">Deployment Altitude</Text>
                        <Text fw={500}>{jump.deploymentAltitude || 'N/A'} ft</Text>
                      </Grid.Col>
                      <Grid.Col span={6}>
                        <Text size="sm" c="dimmed">Freefall Time</Text>
                        <Text fw={500}>{jump.freefallTime} seconds</Text>
                      </Grid.Col>
                      <Grid.Col span={6}>
                        <Text size="sm" c="dimmed">Average Fall Rate</Text>
                        <Text fw={500}>{jump.averageFallRate} mph</Text>
                      </Grid.Col>
                      {jump.maxSpeed && (
                        <Grid.Col span={6}>
                          <Text size="sm" c="dimmed">Max Speed</Text>
                          <Text fw={500}>{jump.maxSpeed} mph</Text>
                        </Grid.Col>
                      )}
                    </Grid>
                  ) : (
                    <Text c="dimmed">Analysis pending...</Text>
                  )}
                </Paper>
              </Stack>
            </Grid.Col>

            <Grid.Col span={{ base: 12, md: 4 }}>
              <Paper p="md" withBorder>
                <Group justify="space-between" mb="md">
                  <Title order={4}>Notes</Title>
                  {jump.isOwner && (
                    <ActionIcon
                      variant="subtle"
                      onClick={() => setNotesModalOpened(true)}
                      title="Edit notes"
                    >
                      <IconEdit size={16} />
                    </ActionIcon>
                  )}
                </Group>
                {jump.notes ? (
                  <div style={{
                    maxHeight: '300px',
                    overflowY: 'auto',
                    fontSize: '14px',
                  }}>
                    <ReactMarkdown>{jump.notes}</ReactMarkdown>
                  </div>
                ) : (
                  <Text c="dimmed">
                    {jump.isOwner ? 'No notes added yet. Click edit to add notes.' : 'No notes'}
                  </Text>
                )}
              </Paper>
            </Grid.Col>
          </Grid>
          <EditNotesModal
            opened={notesModalOpened}
            onClose={() => setNotesModalOpened(false)}
            jumpId={jump.id}
            currentNotes={jump.notes}
            onSuccess={(notes) => {
              setJump(prev => prev ? { ...prev, notes } : null);
            }}
          />
        </Container>
      </AppLayout>
    </AuthGuard>
  );
}
