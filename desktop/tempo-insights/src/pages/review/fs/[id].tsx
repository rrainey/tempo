import { useEffect, useState, useRef } from 'react';
import { useRouter } from 'next/router';
import {
  Container,
  Title,
  Paper,
  Text,
  Group,
  Button,
  Stack,
  Badge,
  Card,
  Grid,
  Select
} from '@mantine/core';
import { AppLayout } from '../../../components/AppLayout';
import { AuthGuard } from '../../../components/AuthGuard';
import { IconPlayerPlay, IconPlayerPause, IconReload } from '@tabler/icons-react';

interface Participant {
  id: string;
  name: string;
  position: number;
  color: string;
}

interface FormationSkydive {
  id: string;
  name: string;
  jumpTime: string;
  altitude: number;
  aircraft: string;
  participants: Participant[];
}

export default function FormationReviewPage() {
  const router = useRouter();
  const { id } = router.query;
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [formation, setFormation] = useState<FormationSkydive | null>(null);
  const [loading, setLoading] = useState(true);
  const [playing, setPlaying] = useState(false);
  const [currentTime, setCurrentTime] = useState(0);
  const [viewMode, setViewMode] = useState<string>('3d');

  useEffect(() => {
    if (id) {
      loadFormationData();
    }
  }, [id]);

  useEffect(() => {
    if (formation && canvasRef.current) {
      initializeCanvas();
    }
  }, [formation, viewMode]);

  const loadFormationData = async () => {
    try {
      // TODO: Implement formation API endpoint
      // const response = await fetch(`/api/formations/${id}`);
      // const data = await response.json();
      // setFormation(data.formation);

      // Mock data for now
      if (id === 'test-formation') {
        setFormation({
          id: 'test-formation',
          name: '4-Way Test Jump',
          jumpTime: new Date().toISOString(),
          altitude: 13500,
          aircraft: 'Twin Otter',
          participants: [
            { id: '1', name: 'Alice', position: 1, color: '#FF6B6B' },
            { id: '2', name: 'Bob', position: 2, color: '#4ECDC4' },
            { id: '3', name: 'Charlie', position: 3, color: '#45B7D1' },
            { id: '4', name: 'Diana', position: 4, color: '#96CEB4' },
          ],
        });
      }
      setLoading(false);
    } catch (err) {
      setLoading(false);
    }
  };

  const initializeCanvas = () => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // Set canvas size
    canvas.width = canvas.offsetWidth;
    canvas.height = 500;

    // Clear canvas
    ctx.fillStyle = '#002233';
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    // Draw placeholder visualization
    ctx.strokeStyle = '#ddff55';
    ctx.lineWidth = 2;
    ctx.strokeRect(50, 50, canvas.width - 100, canvas.height - 100);

    ctx.fillStyle = '#ddffee';
    ctx.font = '18px sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('3D Formation Visualization Placeholder', canvas.width / 2, canvas.height / 2);
    ctx.font = '14px sans-serif';
    ctx.fillText(`View Mode: ${viewMode}`, canvas.width / 2, canvas.height / 2 + 30);
  };

  const handlePlayPause = () => {
    setPlaying(!playing);
    // TODO: Implement playback logic
  };

  const handleReset = () => {
    setPlaying(false);
    setCurrentTime(0);
    // TODO: Reset visualization
  };

  if (loading) return (
    <AuthGuard>
      <AppLayout>
        <Container>
          <Text>Loading formation data...</Text>
        </Container>
      </AppLayout>
    </AuthGuard>
  );

  if (!formation) return (
    <AuthGuard>
      <AppLayout>
        <Container>
          <Text c="red">Formation not found</Text>
        </Container>
      </AppLayout>
    </AuthGuard>
  );

  return (
    <AuthGuard>
      <AppLayout>
        <Container fluid>
          <Group justify="space-between" mb="md">
            <div>
              <Title order={2}>{formation.name}</Title>
              <Text c="dimmed">
                {new Date(formation.jumpTime).toLocaleString()} •
                {formation.altitude}ft • {formation.aircraft}
              </Text>
            </div>
            <Select
              value={viewMode}
              onChange={(value) => setViewMode(value || '3d')}
              data={[
                { value: '3d', label: '3D View' },
                { value: 'top', label: 'Top View' },
                { value: 'side', label: 'Side View' },
              ]}
            />
          </Group>

          <Grid>
            <Grid.Col span={{ base: 12, lg: 9 }}>
              <Paper p="md" withBorder>
                <canvas
                  ref={canvasRef}
                  style={{
                    width: '100%',
                    height: '500px',
                    backgroundColor: '#002233',
                    borderRadius: '4px'
                  }}
                />

                <Group justify="center" mt="md">
                  <Button
                    leftSection={playing ? <IconPlayerPause size={16} /> : <IconPlayerPlay size={16} />}
                    onClick={handlePlayPause}
                    color="accent"
                  >
                    {playing ? 'Pause' : 'Play'}
                  </Button>
                  <Button
                    leftSection={<IconReload size={16} />}
                    onClick={handleReset}
                    variant="subtle"
                  >
                    Reset
                  </Button>
                </Group>
              </Paper>
            </Grid.Col>

            <Grid.Col span={{ base: 12, lg: 3 }}>
              <Stack gap="md">
                <Paper p="md" withBorder>
                  <Title order={4} mb="sm">Participants</Title>
                  <Stack gap="xs">
                    {formation.participants.map((participant) => (
                      <Group key={participant.id} justify="space-between">
                        <Group gap="xs">
                          <div
                            style={{
                              width: 16,
                              height: 16,
                              borderRadius: 8,
                              backgroundColor: participant.color
                            }}
                          />
                          <Text size="sm">{participant.name}</Text>
                        </Group>
                        <Badge size="sm">P{participant.position}</Badge>
                      </Group>
                    ))}
                  </Stack>
                </Paper>

                <Paper p="md" withBorder>
                  <Title order={4} mb="sm">Timeline</Title>
                  <Stack gap="xs">
                    <Text size="sm">Exit: 0:00</Text>
                    <Text size="sm">Break-off: 0:45</Text>
                    <Text size="sm">Deployment: 1:00</Text>
                  </Stack>
                </Paper>
              </Stack>
            </Grid.Col>
          </Grid>
        </Container>
      </AppLayout>
    </AuthGuard>
  );
}
