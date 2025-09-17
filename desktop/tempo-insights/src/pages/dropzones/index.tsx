// /pages/dropzones/index.tsx
import { useState, useEffect } from 'react';
import { 
  Container, 
  Title, 
  Table, 
  Button, 
  Group, 
  Badge,
  ActionIcon,
  Modal,
  TextInput,
  NumberInput,
  Textarea,
  Stack,
  Switch,
  Text
} from '@mantine/core';
import { useForm } from '@mantine/form';
import { notifications } from '@mantine/notifications';
import { IconPlus, IconEdit, IconTrash, IconMapPin } from '@tabler/icons-react';
import { AppLayout } from '../../components/AppLayout';
import { AuthGuard } from '../../components/AuthGuard';
import { useRouter } from 'next/router';

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
  _count: {
    formations: number;
  };
}

export default function DropzonesPage() {
  const router = useRouter();
  const [dropzones, setDropzones] = useState<Dropzone[]>([]);
  const [loading, setLoading] = useState(true);
  const [modalOpen, setModalOpen] = useState(false);
  const [editingDropzone, setEditingDropzone] = useState<Dropzone | null>(null);

  const form = useForm({
    initialValues: {
      name: '',
      icaoCode: '',
      latitude: 0,
      longitude: 0,
      elevation: 0,
      timezone: '',
      notes: '',
      isActive: true
    },
    validate: {
      name: (value) => !value ? 'Name is required' : null,
      latitude: (value) => value < -90 || value > 90 ? 'Invalid latitude' : null,
      longitude: (value) => value < -180 || value > 180 ? 'Invalid longitude' : null,
      timezone: (value) => !value ? 'Timezone is required' : null,
    }
  });

  useEffect(() => {
    loadDropzones();
  }, []);

  const loadDropzones = async () => {
    try {
      const response = await fetch('/api/dropzones');
      if (!response.ok) throw new Error('Failed to load dropzones');
      
      const data = await response.json();
      setDropzones(data.dropzones);
    } catch (error) {
      notifications.show({
        title: 'Error',
        message: 'Failed to load dropzones',
        color: 'red'
      });
    } finally {
      setLoading(false);
    }
  };

  const handleSubmit = async (values: typeof form.values) => {
    try {
      const url = editingDropzone 
        ? `/api/dropzones/${editingDropzone.id}`
        : '/api/dropzones';
      
      const method = editingDropzone ? 'PATCH' : 'POST';
      
      const response = await fetch(url, {
        method,
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(values)
      });

      if (!response.ok) {
        const error = await response.json();
        throw new Error(error.error || 'Failed to save dropzone');
      }

      notifications.show({
        title: 'Success',
        message: editingDropzone ? 'Dropzone updated' : 'Dropzone created',
        color: 'green'
      });

      setModalOpen(false);
      form.reset();
      setEditingDropzone(null);
      loadDropzones();
    } catch (error) {
      notifications.show({
        title: 'Error',
        message: error instanceof Error ? error.message : 'Failed to save dropzone',
        color: 'red'
      });
    }
  };

  const handleEdit = (dropzone: Dropzone) => {
    setEditingDropzone(dropzone);
    form.setValues({
      name: dropzone.name,
      icaoCode: dropzone.icaoCode || '',
      latitude: dropzone.latitude,
      longitude: dropzone.longitude,
      elevation: dropzone.elevation,
      timezone: dropzone.timezone,
      notes: '',
      isActive: dropzone.isActive
    });
    setModalOpen(true);
  };

  const handleDelete = async (dropzone: Dropzone) => {
    if (dropzone._count.formations > 0) {
      notifications.show({
        title: 'Cannot Delete',
        message: 'This dropzone has associated formations',
        color: 'orange'
      });
      return;
    }

    if (!confirm(`Delete dropzone "${dropzone.name}"?`)) return;

    try {
      const response = await fetch(`/api/dropzones/${dropzone.id}`, {
        method: 'DELETE'
      });

      if (!response.ok) throw new Error('Failed to delete dropzone');

      notifications.show({
        title: 'Success',
        message: 'Dropzone deleted',
        color: 'green'
      });

      loadDropzones();
    } catch (error) {
      notifications.show({
        title: 'Error',
        message: 'Failed to delete dropzone',
        color: 'red'
      });
    }
  };

  return (
    <AuthGuard requireAdmin>
      <AppLayout>
        <Container>
          <Group justify="space-between" mb="xl">
            <Title order={2}>Dropzones</Title>
            <Button 
              leftSection={<IconPlus size={16} />}
              onClick={() => {
                form.reset();
                setEditingDropzone(null);
                setModalOpen(true);
              }}
            >
              Add Dropzone
            </Button>
          </Group>

          <Table striped>
            <Table.Thead>
              <Table.Tr>
                <Table.Th>Name</Table.Th>
                <Table.Th>ICAO</Table.Th>
                <Table.Th>Location</Table.Th>
                <Table.Th>Elevation</Table.Th>
                <Table.Th>Formations</Table.Th>
                <Table.Th>Status</Table.Th>
                <Table.Th>Actions</Table.Th>
              </Table.Tr>
            </Table.Thead>
            <Table.Tbody>
              {dropzones.map((dz) => (
                <Table.Tr key={dz.id}>
                  <Table.Td>
                    <Group gap="xs">
                      <IconMapPin size={16} />
                      <Text 
                        component="a"
                        href={`/dropzones/${dz.slug}`}
                        style={{ cursor: 'pointer' }}
                      >
                        {dz.name}
                      </Text>
                    </Group>
                  </Table.Td>
                  <Table.Td>{dz.icaoCode || '-'}</Table.Td>
                  <Table.Td>
                    {dz.latitude.toFixed(4)}°, {dz.longitude.toFixed(4)}°
                  </Table.Td>
                  <Table.Td>{Math.round(dz.elevation * 3.28084)} ft</Table.Td>
                  <Table.Td>{dz._count.formations}</Table.Td>
                  <Table.Td>
                    <Badge color={dz.isActive ? 'green' : 'gray'}>
                      {dz.isActive ? 'Active' : 'Inactive'}
                    </Badge>
                  </Table.Td>
                  <Table.Td>
                    <Group gap="xs">
                      <ActionIcon 
                        variant="subtle"
                        onClick={() => handleEdit(dz)}
                      >
                        <IconEdit size={16} />
                      </ActionIcon>
                      <ActionIcon 
                        variant="subtle"
                        color="red"
                        onClick={() => handleDelete(dz)}
                        disabled={dz._count.formations > 0}
                      >
                        <IconTrash size={16} />
                      </ActionIcon>
                    </Group>
                  </Table.Td>
                </Table.Tr>
              ))}
            </Table.Tbody>
          </Table>

          <Modal
            opened={modalOpen}
            onClose={() => setModalOpen(false)}
            title={editingDropzone ? 'Edit Dropzone' : 'Add Dropzone'}
            size="lg"
          >
            <form onSubmit={form.onSubmit(handleSubmit)}>
              <Stack>
                <TextInput
                  label="Name"
                  required
                  {...form.getInputProps('name')}
                />
                <TextInput
                  label="ICAO Code"
                  placeholder="e.g., KSDM"
                  {...form.getInputProps('icaoCode')}
                />
                <Group grow>
                  <NumberInput
                    label="Latitude"
                    required
                    min={-90}
                    max={90}
                    {...form.getInputProps('latitude')}
                  />
                  <NumberInput
                    label="Longitude"
                    required
                    min={-180}
                    max={180}
                    {...form.getInputProps('longitude')}
                  />
                </Group>
                <NumberInput
                  label="Elevation (meters MSL)"
                  required
                  {...form.getInputProps('elevation')}
                />
                <TextInput
                  label="Timezone"
                  required
                  placeholder="e.g., America/Los_Angeles"
                  {...form.getInputProps('timezone')}
                />
                <Textarea
                  label="Notes"
                  {...form.getInputProps('notes')}
                />
                {editingDropzone && (
                  <Switch
                    label="Active"
                    {...form.getInputProps('isActive', { type: 'checkbox' })}
                  />
                )}
                <Group justify="flex-end">
                  <Button variant="subtle" onClick={() => setModalOpen(false)}>
                    Cancel
                  </Button>
                  <Button type="submit">
                    {editingDropzone ? 'Update' : 'Create'}
                  </Button>
                </Group>
              </Stack>
            </form>
          </Modal>
        </Container>
      </AppLayout>
    </AuthGuard>
  );
}