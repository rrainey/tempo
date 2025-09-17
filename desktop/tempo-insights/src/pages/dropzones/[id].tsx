// /pages/dropzones/[slug].tsx
import { useState, useEffect } from 'react';
import { useRouter } from 'next/router';
import { 
  Container, 
  Title, 
  Paper, 
  Stack, 
  Group, 
  Text,
  Badge,
  SimpleGrid,
  Card
} from '@mantine/core';
import { IconMapPin, IconPlane, IconClock } from '@tabler/icons-react';
import { AppLayout } from '../../components/AppLayout';
import { AuthGuard } from '../../components/AuthGuard';

interface Dropzone {
  id: string;
  name: string;
  slug: string;
  icaoCode: string | null;
  latitude: number;
  longitude: number;
  elevation: number;
  timezone: string;
  isActive: boolean;
  notes: string | null;
  _count: {
    formations: number;
  };
}

export default function DropzoneDetailPage() {
  const router = useRouter();
  const { slug } = router.query;
  const [dropzone, setDropzone] = useState<Dropzone | null>(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    if (slug) {
      loadDropzone();
    }
  }, [slug]);

  const loadDropzone = async () => {
    try {
      const response = await fetch(`/api/dropzones/${slug}`);
      if (!response.ok) throw new Error('Failed to load dropzone');
      
      const data = await response.json();
      setDropzone(data.dropzone);
    } catch (error) {
      console.error('Error loading dropzone:', error);
    } finally {
      setLoading(false);
    }
  };

  if (loading) return <div>Loading...</div>;
  if (!dropzone) return <div>Dropzone not found</div>;

  return (
    <AuthGuard>
      <AppLayout>
        <Container>
          <Group justify="space-between" mb="xl">
            <Title order={2}>
              <Group gap="sm">
                <IconMapPin size={32} />
                {dropzone.name}
              </Group>
            </Title>
            <Badge size="lg" color={dropzone.isActive ? 'green' : 'gray'}>
              {dropzone.isActive ? 'Active' : 'Inactive'}
            </Badge>
          </Group>

          <SimpleGrid cols={{ base: 1, md: 2 }} spacing="md">
            <Card withBorder>
              <Title order={4} mb="md">Location Details</Title>
              <Stack gap="sm">
                {dropzone.icaoCode && (
                  <Group>
                    <IconPlane size={16} />
                    <Text fw={500}>ICAO:</Text>
                    <Text>{dropzone.icaoCode}</Text>
                  </Group>
                )}
                <Group>
                  <Text fw={500}>Coordinates:</Text>
                  <Text>
                    {dropzone.latitude.toFixed(6)}°N, {dropzone.longitude.toFixed(6)}°E
                  </Text>
                </Group>
                <Group>
                  <Text fw={500}>Elevation:</Text>
                  <Text>
                    {Math.round(dropzone.elevation)} m MSL 
                    ({Math.round(dropzone.elevation * 3.28084)} ft)
                  </Text>
                </Group>
                <Group>
                  <IconClock size={16} />
                  <Text fw={500}>Timezone:</Text>
                  <Text>{dropzone.timezone}</Text>
                </Group>
              </Stack>
            </Card>

            <Card withBorder>
              <Title order={4} mb="md">Statistics</Title>
              <Stack gap="sm">
                <Group>
                  <Text fw={500}>Total Formations:</Text>
                  <Text>{dropzone._count.formations}</Text>
                </Group>
                {dropzone.notes && (
                  <div>
                    <Text fw={500} mb="xs">Notes:</Text>
                    <Text size="sm" c="dimmed">{dropzone.notes}</Text>
                  </div>
                )}
              </Stack>
            </Card>
          </SimpleGrid>

          {/* Map could go here in the future */}
          <Paper withBorder p="md" mt="xl">
            <Text c="dimmed" ta="center">
              Map visualization will be added in a future update
            </Text>
          </Paper>
        </Container>
      </AppLayout>
    </AuthGuard>
  );
}