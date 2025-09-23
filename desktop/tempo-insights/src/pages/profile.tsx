import { useState, useEffect } from 'react';
import { Container, Title, Paper, TextInput, Button, Group, Text, Alert, Stack, Card, Select, NumberInput } from '@mantine/core';
import { useForm } from '@mantine/form';
import { notifications } from '@mantine/notifications';
import { AppLayout } from '../components/AppLayout';
import { AuthGuard } from '../components/AuthGuard';
import { ChangePasswordModal } from '../components/ChangePasswordModal';
import { IconLock, IconParachute, IconMapPin } from '@tabler/icons-react';

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

export default function ProfilePage() {
  const [loading, setLoading] = useState(false);
  const [user, setUser] = useState<UserProfile | null>(null);
  const [dropzones, setDropzones] = useState<Dropzone[]>([]);
  const [error, setError] = useState<string | null>(null);
  const [passwordModalOpened, setPasswordModalOpened] = useState(false);

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
                  <Button
                    variant="subtle"
                    leftSection={<IconLock size={16} />}
                    onClick={() => setPasswordModalOpened(true)}
                  >
                    Change Password
                  </Button>

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
      </AppLayout>
    </AuthGuard>
  );
}