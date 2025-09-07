import { useState, useEffect } from 'react';
import { Container, Title, Paper, TextInput, Button, Group, Text, Alert } from '@mantine/core';
import { useForm } from '@mantine/form';
import { notifications } from '@mantine/notifications';
import { AppLayout } from '../components/AppLayout';
import { AuthGuard } from '../components/AuthGuard';

interface ProfileForm {
  name: string;
  email: string;
}

interface User {
  id: string;
  email: string;
  name: string;
  slug: string;
  role: string;
}

export default function ProfilePage() {
  const [loading, setLoading] = useState(false);
  const [user, setUser] = useState<User | null>(null);
  const [error, setError] = useState<string | null>(null);

  const form = useForm<ProfileForm>({
    initialValues: {
      name: '',
      email: '',
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
    },
  });

  useEffect(() => {
    loadProfile();
  }, []);

  const loadProfile = async () => {
    try {
      const response = await fetch('/api/auth/me');
      if (!response.ok) throw new Error('Failed to load profile');

      const data = await response.json();
      setUser(data.user);
      form.setValues({
        name: data.user.name,
        email: data.user.email,
      });
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to load profile');
    }
  };

  const handleSubmit = async (values: ProfileForm) => {
    setLoading(true);
    setError(null);

    try {
      // TODO: Implement profile update API endpoint
      // const response = await fetch('/api/profile/update', {
      //   method: 'PUT',
      //   headers: { 'Content-Type': 'application/json' },
      //   body: JSON.stringify(values),
      // });

      // For now, just show a notification
      notifications.show({
        title: 'Profile Update',
        message: 'Profile update API not yet implemented',
        color: 'blue',
      });
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred');
    } finally {
      setLoading(false);
    }
  };

  return (
    <AuthGuard>
      <AppLayout>
        <Container size="sm">
          <Title order={2} mb="xl">My Profile</Title>

          <Paper p="xl" withBorder>
            {error && (
              <Alert color="red" mb="md">
                {error}
              </Alert>
            )}

            {user && (
              <Group mb="lg">
                <div>
                  <Text size="sm" c="dimmed">User ID</Text>
                  <Text fw={500}>{user.id}</Text>
                </div>
                <div>
                  <Text size="sm" c="dimmed">Slug</Text>
                  <Text fw={500}>@{user.slug}</Text>
                </div>
                <div>
                  <Text size="sm" c="dimmed">Role</Text>
                  <Text fw={500}>{user.role}</Text>
                </div>
              </Group>
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

              <Group justify="flex-end" mt="xl">
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
            </form>
          </Paper>
        </Container>
      </AppLayout>
    </AuthGuard>
  );
}
