import { useState, useEffect } from 'react';
import { Container, Title, Paper, TextInput, Button, Group, Text, Alert, Stack, Card, Select, NumberInput, Modal } from '@mantine/core';
import { useForm } from '@mantine/form';
import { notifications } from '@mantine/notifications';
import { AppLayout } from '../components/AppLayout';
import { AuthGuard } from '../components/AuthGuard';
import { ChangePasswordModal } from '../components/ChangePasswordModal';
import { IconLock, IconParachute, IconMapPin, IconDownload, IconFileZip } from '@tabler/icons-react';

interface ProfileForm {
  name: string;
  email: string;
  nextJumpNumber: number;
  homeDropzoneId: string | null;
}

interface UserProfile {
  id: string;
  email: string;
  name: string;
  slug: string;
  role: string;
  nextJumpNumber: number;
  homeDropzoneId: string | null;
  homeDropzone: {
    id: string;
    name: string;
    slug: string;
  } | null;
  createdAt: string;
  jumpCount: number;
  groupCount: number;
  deviceCount: number;
}

interface Dropzone {
  id: string;
  name: string;
  slug: string;
  icaoCode: string | null;
}

interface ExportSizeInfo {
  jumpCount: number;
  totalRawLogSize: number;
  estimatedMetadataSize: number;
  estimatedTotalSize: number;
  estimatedTotalSizeMB: string;
}

export default function ProfilePage() {
  const [loading, setLoading] = useState(false);
  const [user, setUser] = useState<UserProfile | null>(null);
  const [dropzones, setDropzones] = useState<Dropzone[]>([]);
  const [error, setError] = useState<string | null>(null);
  const [passwordModalOpened, setPasswordModalOpened] = useState(false);
  const [exportModalOpened, setExportModalOpened] = useState(false);
  const [exportSizeInfo, setExportSizeInfo] = useState<ExportSizeInfo | null>(null);
  const [loadingExportSize, setLoadingExportSize] = useState(false);
  const [exportingData, setExportingData] = useState(false);

  const form = useForm<ProfileForm>({
    initialValues: {
      name: '',
      email: '',
      nextJumpNumber: 1,
      homeDropzoneId: null,
    },
    validate: {
      name: (value) => {
        if (!value) return 'Name is required';
        if (value.length < 2) return 'Name must be at least 2 characters';
        return null;
      },
      email: (value) => {
        if (!value) return 'Email is required';
        if (!/^\S+@\S+$/.test(value)) return 'Invalid email';
        return null;
      },
      nextJumpNumber: (value) => {
        if (!value || value < 1) return 'Jump number must be at least 1';
        if (!Number.isInteger(value)) return 'Jump number must be a whole number';
        return null;
      },
    },
  });

  useEffect(() => {
    loadProfile();
    loadDropzones();
  }, []);

  const loadProfile = async () => {
    try {
      const response = await fetch('/api/users/me');
      if (!response.ok) throw new Error('Failed to load profile');

      const data = await response.json();
      setUser(data.user);
      form.setValues({
        name: data.user.name,
        email: data.user.email,
        nextJumpNumber: data.user.nextJumpNumber,
        homeDropzoneId: data.user.homeDropzoneId,
      });
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to load profile');
    }
  };

  const loadDropzones = async () => {
    try {
      const response = await fetch('/api/dropzones');
      if (!response.ok) throw new Error('Failed to load dropzones');

      const data = await response.json();
      setDropzones(data.dropzones || []);
    } catch (err) {
      console.error('Failed to load dropzones:', err);
      // Not critical if dropzones fail to load
    }
  };

  const handleSubmit = async (values: ProfileForm) => {
    setLoading(true);
    setError(null);

    try {
      const response = await fetch('/api/users/me', {
        method: 'PATCH',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(values),
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.error || 'Failed to update profile');
      }

      setUser(data.user);
      notifications.show({
        title: 'Success',
        message: 'Profile updated successfully',
        color: 'green',
      });
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred');
    } finally {
      setLoading(false);
    }
  };

  const handleRequestExport = async () => {
    setLoadingExportSize(true);
    setExportSizeInfo(null);

    try {
      const response = await fetch('/api/export/size');
      if (!response.ok) throw new Error('Failed to get export size');

      const data = await response.json();
      setExportSizeInfo(data);
      setExportModalOpened(true);
    } catch (err) {
      notifications.show({
        title: 'Error',
        message: err instanceof Error ? err.message : 'Failed to estimate export size',
        color: 'red',
      });
    } finally {
      setLoadingExportSize(false);
    }
  };

  const handleConfirmExport = async () => {
    setExportingData(true);

    try {
      // Create a hidden link and trigger download
      const link = document.createElement('a');
      link.href = '/api/export';
      link.download = `tempo-export-${Date.now()}.zip`;
      document.body.appendChild(link);
      link.click();
      document.body.removeChild(link);

      notifications.show({
        title: 'Export Started',
        message: 'Your data export has started. The download will begin shortly.',
        color: 'green',
      });

      setExportModalOpened(false);
    } catch (err) {
      notifications.show({
        title: 'Error',
        message: err instanceof Error ? err.message : 'Failed to export data',
        color: 'red',
      });
    } finally {
      setExportingData(false);
    }
  };

  const dropzoneOptions = dropzones.map(dz => ({
    value: dz.id,
    label: dz.icaoCode ? `${dz.name} (${dz.icaoCode})` : dz.name,
  }));

  // Add "None" option at the beginning
  const dropzoneSelectData = [
    { value: '', label: 'None' },
    ...dropzoneOptions,
  ];

  return (
    <AuthGuard>
      <AppLayout>
        <Container size="sm">
          <Title order={2} mb="xl">My Profile</Title>

          <Stack gap="lg">
            {/* Stats Cards */}
            {user && (
              <Group grow>
                <Card shadow="sm" p="md" withBorder>
                  <Text size="sm" c="dimmed" mb={5}>
                    Total Jumps
                  </Text>
                  <Text size="xl" fw={600}>
                    {user.jumpCount}
                  </Text>
                </Card>
                
                <Card shadow="sm" p="md" withBorder>
                  <Text size="sm" c="dimmed" mb={5}>
                    Next Jump Number
                  </Text>
                  <Group gap="xs">
                    <IconParachute size={20} />
                    <Text size="xl" fw={600}>
                      #{user.nextJumpNumber}
                    </Text>
                  </Group>
                </Card>
                
                <Card shadow="sm" p="md" withBorder>
                  <Text size="sm" c="dimmed" mb={5}>
                    Groups
                  </Text>
                  <Text size="xl" fw={600}>
                    {user.groupCount}
                  </Text>
                </Card>
                
                <Card shadow="sm" p="md" withBorder>
                  <Text size="sm" c="dimmed" mb={5}>
                    Devices
                  </Text>
                  <Text size="xl" fw={600}>
                    {user.deviceCount}
                  </Text>
                </Card>
              </Group>
            )}

            {/* Profile Form */}
            <Paper p="xl" withBorder>
              <Title order={3} mb="lg">Profile Information</Title>

              {error && (
                <Alert color="red" mb="md">
                  {error}
                </Alert>
              )}

              <form onSubmit={form.onSubmit(handleSubmit)}>
                <TextInput
                  label="Name"
                  placeholder="Your name"
                  required
                  mb="md"
                  {...form.getInputProps('name')}
                />

                <TextInput
                  label="Email"
                  placeholder="your@email.com"
                  required
                  mb="md"
                  {...form.getInputProps('email')}
                />

                <NumberInput
                  label="Next Jump Number"
                  description="Your next jump will be logged with this number"
                  placeholder="1"
                  min={1}
                  required
                  mb="md"
                  leftSection={<IconParachute size={16} />}
                  {...form.getInputProps('nextJumpNumber')}
                />

                <Select
                  label="Home Dropzone"
                  description="Your default dropzone for jump analysis"
                  placeholder="Select a dropzone"
                  leftSection={<IconMapPin size={16} />}
                  data={dropzoneSelectData}
                  searchable
                  clearable
                  mb="md"
                  value={form.values.homeDropzoneId || ''}
                  onChange={(value) => form.setFieldValue('homeDropzoneId', value || null)}
                  error={form.errors.homeDropzoneId}
                />

                {user && (
                  <Stack gap="xs" mb="md">
                    <Text size="sm" c="dimmed">
                      Profile URL: /users/{user.slug}
                    </Text>
                    <Text size="sm" c="dimmed">
                      Member since: {new Date(user.createdAt).toLocaleDateString()}
                    </Text>
                  </Stack>
                )}

                <Group justify="space-between" mt="xl">
                  <Group>
                    <Button
                      variant="subtle"
                      leftSection={<IconLock size={16} />}
                      onClick={() => setPasswordModalOpened(true)}
                    >
                      Change Password
                    </Button>
                    <Button
                      variant="subtle"
                      leftSection={<IconDownload size={16} />}
                      onClick={handleRequestExport}
                      loading={loadingExportSize}
                    >
                      Export My Data
                    </Button>
                  </Group>

                  <Group>
                    <Button
                      variant="subtle"
                      onClick={() => loadProfile()}
                      disabled={loading}
                    >
                      Cancel
                    </Button>
                    <Button
                      type="submit"
                      loading={loading}
                      disabled={loading}
                      color="brand"
                    >
                      Save Changes
                    </Button>
                  </Group>
                </Group>
              </form>
            </Paper>
          </Stack>
        </Container>

        <ChangePasswordModal
          opened={passwordModalOpened}
          onClose={() => setPasswordModalOpened(false)}
        />

        <Modal
          opened={exportModalOpened}
          onClose={() => setExportModalOpened(false)}
          title="Export My Data"
          size="md"
        >
          <Stack gap="md">
            <Alert color="blue" icon={<IconFileZip size={16} />}>
              This will export all your jump data in a ZIP archive containing:
              <ul style={{ marginTop: 8, marginBottom: 0 }}>
                <li>All jump logs (raw NMEA format files)</li>
                <li>Jump analysis metadata (JSON format)</li>
                <li>Your profile information</li>
                <li>Group memberships</li>
                <li>Formation skydive information</li>
                <li>Device information</li>
                <li>Connection list</li>
              </ul>
            </Alert>

            {exportSizeInfo && (
              <Paper p="md" withBorder>
                <Text size="sm" fw={600} mb="xs">Export Details:</Text>
                <Stack gap="xs">
                  <Group justify="space-between">
                    <Text size="sm" c="dimmed">Jump logs:</Text>
                    <Text size="sm">{exportSizeInfo.jumpCount}</Text>
                  </Group>
                  <Group justify="space-between">
                    <Text size="sm" c="dimmed">Estimated size:</Text>
                    <Text size="sm" fw={600}>{exportSizeInfo.estimatedTotalSizeMB} MB</Text>
                  </Group>
                </Stack>
              </Paper>
            )}

            <Text size="sm" c="dimmed">
              The download may take a few moments depending on the amount of data.
            </Text>

            <Group justify="flex-end" mt="md">
              <Button
                variant="subtle"
                onClick={() => setExportModalOpened(false)}
                disabled={exportingData}
              >
                Cancel
              </Button>
              <Button
                onClick={handleConfirmExport}
                loading={exportingData}
                leftSection={<IconDownload size={16} />}
                color="brand"
              >
                Download Export
              </Button>
            </Group>
          </Stack>
        </Modal>
      </AppLayout>
    </AuthGuard>
  );
}